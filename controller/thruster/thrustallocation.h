/*
*******************************************************************************
* thrustallocation.h:
* function for control allocation based on Quadratic programming, using
* Mosek solver API. Normally, thrust alloation is used in the fully-actuated
* control system.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _THRUSTALLOCATION_H_
#define _THRUSTALLOCATION_H_

#include <math.h>
#include <stdio.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// #include "common/logging/include/easylogging++.h"
#include "controllerdata.h"
#include "mosek.h"

// m: # of all thrusters on the vessel
// n: # of dimension of control space
template <int m, ACTUATION index_actuation, int n = 3>
class thrustallocation {
  using vectormd = Eigen::Matrix<double, m, 1>;
  using vectormi = Eigen::Matrix<int, m, 1>;
  using vectornd = Eigen::Matrix<double, n, 1>;
  using matrixnmd = Eigen::Matrix<double, n, m>;
  using matrixmmd = Eigen::Matrix<double, m, m>;
  using matrixnnd = Eigen::Matrix<double, n, n>;

 public:
  explicit thrustallocation(
      const thrustallocationdata &_thrustallocationdata,
      const std::vector<tunnelthrusterdata> &_v_tunnelthrusterdata,
      const std::vector<azimuththrusterdata> &_v_azimuththrusterdata,
      const std::vector<ruddermaindata> &_v_ruddermaindata,
      const std::vector<twinfixedthrusterdata> &_v_twinfixeddata)
      : Q_surge(_thrustallocationdata.Q_surge),
        Q_sway(_thrustallocationdata.Q_sway),
        Q_yaw(_thrustallocationdata.Q_yaw),
        num_tunnel(_thrustallocationdata.num_tunnel),
        num_azimuth(_thrustallocationdata.num_azimuth),
        num_mainrudder(_thrustallocationdata.num_mainrudder),
        num_twinfixed(_thrustallocationdata.num_twinfixed),
        numvar(2 * m + n),
        num_constraints(n),
        index_thrusters(_thrustallocationdata.index_thrusters),
        v_tunnelthrusterdata(_v_tunnelthrusterdata),
        v_azimuththrusterdata(_v_azimuththrusterdata),
        v_ruddermaindata(_v_ruddermaindata),
        v_twinfixeddata(_v_twinfixeddata),
        lx(vectormd::Zero()),
        ly(vectormd::Zero()),
        upper_delta_alpha(vectormd::Zero()),
        lower_delta_alpha(vectormd::Zero()),
        upper_delta_u(vectormd::Zero()),
        lower_delta_u(vectormd::Zero()),
        Q(matrixnnd::Zero()),
        Omega(matrixmmd::Zero()),
        Q_deltau(matrixmmd::Zero()),
        g_deltau(vectormd::Zero()),
        d_rho(vectormd::Zero()),
        B_alpha(matrixnmd::Zero()),
        d_Balpha_u(matrixnmd::Zero()),
        b(vectornd::Zero()),
        delta_alpha(vectormd::Zero()),
        delta_u(vectormd::Zero()),
        derivative_dx(1e-6),
        results(Eigen::Matrix<double, 2 * m + n, 1>::Zero()) {
    initializethrusterallocation();
  }

  thrustallocation() = delete;
  ~thrustallocation() {
    MSK_deletetask(&task);
    MSK_deleteenv(&env);
  }

  // perform the thrust allocation using QP solver (one step)
  void onestepthrustallocation(controllerRTdata<m, n> &_RTdata) {
    update_formerstep_feedback(_RTdata);
    updateTAparameters(_RTdata);
    updateMosekparameters();
    onestepmosek();
    update_nextstep_command(_RTdata);
  }  // onestepthrustallocation

  void initializapropeller(controllerRTdata<m, n> &_RTdata) {
    // alpha and thrust of each propeller
    for (int i = 0; i != num_tunnel; ++i) {
      // command
      _RTdata.command_rotation(i) = 1;
      _RTdata.command_u(i) = v_tunnelthrusterdata[i].K_positive;
      _RTdata.command_alpha(i) = M_PI / 2;
      _RTdata.command_alpha_deg(i) = 90;
    }
    for (int j = 0; j != num_azimuth; ++j) {
      int a_index = j + num_tunnel;
      _RTdata.command_rotation(a_index) = v_azimuththrusterdata[j].min_rotation;
      _RTdata.command_u(a_index) = v_azimuththrusterdata[j].min_thrust;
      _RTdata.command_alpha(a_index) = (v_azimuththrusterdata[j].min_alpha +
                                        v_azimuththrusterdata[j].max_alpha) /
                                       2;

      _RTdata.command_alpha_deg(a_index) =
          rad2degree(_RTdata.command_alpha(a_index));
    }
    for (int k = 0; k != num_mainrudder; ++k) {
      int a_index = k + num_tunnel + num_azimuth;
      _RTdata.command_rotation(a_index) = v_ruddermaindata[k].min_rotation;
      _RTdata.command_u(a_index) = v_ruddermaindata[k].min_thrust;
      _RTdata.command_alpha(a_index) = 0;
      _RTdata.command_alpha_deg(a_index) = 0;
    }
    for (int l = 0; l != num_twinfixed; ++l) {
      int a_index = l + num_tunnel + num_azimuth + num_mainrudder;
      _RTdata.command_rotation(a_index) = 1;
      _RTdata.command_u(a_index) = v_twinfixeddata[l].K_positive;
      _RTdata.command_alpha(a_index) = 0;
      _RTdata.command_alpha_deg(a_index) = 0;
    }
    // update BalphaU
    _RTdata.BalphaU =
        calculateBalphau(_RTdata.command_alpha, _RTdata.command_u);
  }  // initializapropeller

  // modify penality for each error (heading-only controller)
  void setQ(CONTROLMODE _cm) {
    if constexpr (index_actuation == ACTUATION::FULLYACTUATED) {
      switch (_cm) {
        case CONTROLMODE::MANUAL:
          Q(0, 0) = Q_surge;
          Q(1, 1) = Q_sway;
          Q(2, 2) = Q_yaw;
          break;
        case CONTROLMODE::HEADINGONLY:
          // empirical value
          Q(0, 0) = 0.2 * Q_surge;
          Q(1, 1) = 0.2 * Q_sway;
          Q(2, 2) = 2 * Q_yaw;
          break;
        case CONTROLMODE::MANEUVERING:
          Q(0, 0) = Q_surge;
          Q(1, 1) = 0;  // The penalty for sway error is zero
          Q(2, 2) = Q_yaw;
          break;
        case CONTROLMODE::DYNAMICPOSITION:
          Q(0, 0) = Q_surge;
          Q(1, 1) = Q_sway;
          Q(2, 2) = Q_yaw;
          // Q(0, 0) = 500;
          // Q(1, 1) = 500;
          // Q(2, 2) = 1000;
          break;
        default:
          break;
      }
    } else {  // underactuated
      switch (_cm) {
        case CONTROLMODE::MANUAL:
          Q(0, 0) = Q_surge;
          Q(1, 1) = 0;  // The penalty for sway error is zero
          Q(2, 2) = Q_yaw;
          break;
        case CONTROLMODE::HEADINGONLY:
          Q(0, 0) = 0.2 * Q_surge;
          Q(1, 1) = 0;  // The penalty for sway error is zero
          Q(2, 2) = 2 * Q_yaw;
          break;
        case CONTROLMODE::MANEUVERING:
          // Q(0, 0) = 10;  // 取值与螺旋桨最大推力呈负相关
          // // Q(1, 1) = 0;  The penalty for sway error is zero
          // Q(2, 2) = 20;

          // 取值与螺旋桨最大推力呈负相关
          Q(0, 0) = Q_surge;
          Q(1, 1) = 0;  // The penalty for sway error is zero
          Q(2, 2) = Q_yaw;

          break;
        default:
          break;
      }
    }
    // update mosek
    for (int j = 0; j != n; ++j) {
      qval[j + 2 * m] = Q(j, j);
    }
  }  // setQ

  //
  vectormd getlx() const { return lx; }
  vectormd getly() const { return ly; }
  vectormd getupper_delta_alpha() const { return upper_delta_alpha; }
  vectormd getlower_delta_alpha() const { return lower_delta_alpha; }
  vectormd getupper_delta_u() const { return upper_delta_u; }
  vectormd getlower_delta_u() const { return lower_delta_u; }
  matrixnnd getQ() const { return Q; }
  matrixmmd getOmega() const { return Omega; }
  matrixmmd getQ_deltau() const { return Q_deltau; }
  vectormd getg_deltau() const { return g_deltau; }
  vectormd getd_rho() const { return d_rho; }
  matrixnmd getB_alpha() const { return B_alpha; }
  matrixnmd getd_Balpha_u() const { return d_Balpha_u; }
  vectornd getb() const { return b; }
  vectormd getdelta_alpha() const { return delta_alpha; }
  vectormd getdelta_u() const { return delta_u; }
  Eigen::Matrix<double, 2 * m + n, 1> getresults() const { return results; }

 private:
  const double Q_surge;
  const double Q_sway;
  const double Q_yaw;

  const int num_tunnel;
  const int num_azimuth;
  const int num_mainrudder;
  const int num_twinfixed;
  const int numvar;
  const int num_constraints;

  // types of each thruster
  std::vector<int> index_thrusters;
  // constant value of all tunnel thrusters
  std::vector<tunnelthrusterdata> v_tunnelthrusterdata;
  // constant value of all tunnel thrusters
  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  // constant value of all tunnel thrusters
  std::vector<ruddermaindata> v_ruddermaindata;
  // constant value of all twin fixed thrusters
  std::vector<twinfixedthrusterdata> v_twinfixeddata;
  // location of each thruster
  vectormd lx;
  vectormd ly;
  // real time constraints of each thruster
  vectormd upper_delta_alpha;
  vectormd lower_delta_alpha;
  vectormd upper_delta_u;
  vectormd lower_delta_u;
  // quadratic objective
  matrixnnd Q;
  matrixmmd Omega;
  matrixmmd Q_deltau;
  // linear objective
  vectormd g_deltau;
  vectormd d_rho;

  // real time constraint matrix in QP (equality constraint)
  matrixnmd B_alpha;
  matrixnmd d_Balpha_u;  // Jocobian matrix of Balpha times u
  vectornd b;

  // real time physical variable in thruster allocation
  vectormd delta_alpha;  // rad
  vectormd delta_u;      // N
  // linearized parameters
  double derivative_dx;  // step size of the derivative

  // parameters for Mosek API
  MSKint32t aptrb[2 * m + n], aptre[2 * m + n], asub[6 * m + n];
  double aval[6 * m + n];
  /* Bounds on constraints. */
  MSKboundkeye bkc[n];
  double blc[n];
  double buc[n];
  /* Bounds on variables. */
  MSKboundkeye bkx[2 * m + n];
  double blx[2 * m + n];
  double bux[2 * m + n];

  // objective g
  double g[2 * m + n];

  // The lower triangular part of the quadratic objective Q matrix in the
  // objective is specified.
  MSKint32t qsubi[2 * m + n];
  MSKint32t qsubj[2 * m + n];
  double qval[2 * m + n];
  // array to store the optimization results
  Eigen::Matrix<double, 2 * m + n, 1> results;

  // mosek environment
  MSKenv_t env = NULL;
  MSKtask_t task = NULL;
  MSKrescodee r;

  void initializethrusterallocation() {
    assert(num_tunnel + num_azimuth + num_mainrudder + num_twinfixed == m);
    if (num_twinfixed > 0) assert(num_twinfixed == 2);

    for (int i = 0; i != num_tunnel; ++i) {
      lx(i) = v_tunnelthrusterdata[i].lx;
      ly(i) = v_tunnelthrusterdata[i].ly;
      Omega(i, i) = 1;
      // re-calculate the max thrust of tunnel thruser
      v_tunnelthrusterdata[i].max_thrust_positive =
          v_tunnelthrusterdata[i].K_positive *
          std::pow(v_tunnelthrusterdata[i].max_rotation, 2);
      v_tunnelthrusterdata[i].max_thrust_negative =
          v_tunnelthrusterdata[i].K_negative *
          std::pow(v_tunnelthrusterdata[i].max_rotation, 2);
    }
    for (int i = 0; i != num_azimuth; ++i) {
      int index_azimuth = num_tunnel + i;
      lx(index_azimuth) = v_azimuththrusterdata[i].lx;
      ly(index_azimuth) = v_azimuththrusterdata[i].ly;
      Omega(index_azimuth, index_azimuth) = 50;
      // re-calculate the max thrust of azimuth thruser
      v_azimuththrusterdata[i].max_thrust =
          v_azimuththrusterdata[i].K *
          std::pow(v_azimuththrusterdata[i].max_rotation, 2);
      v_azimuththrusterdata[i].min_thrust =
          v_azimuththrusterdata[i].K *
          std::pow(v_azimuththrusterdata[i].min_rotation, 2);
    }
    for (int i = 0; i != num_mainrudder; ++i) {
      int index_rudder = num_tunnel + num_azimuth + i;
      lx(index_rudder) = v_ruddermaindata[i].lx;
      ly(index_rudder) = v_ruddermaindata[i].ly;
      Omega(index_rudder, index_rudder) = 50;
      // re-calculate the max thrust of main thruser with rudder
      double Cy = v_ruddermaindata[i].Cy;
      v_ruddermaindata[i].max_thrust =
          v_ruddermaindata[i].K * std::pow(v_ruddermaindata[i].max_rotation, 2);
      v_ruddermaindata[i].min_thrust =
          v_ruddermaindata[i].K * std::pow(v_ruddermaindata[i].min_rotation, 2);
      v_ruddermaindata[i].max_alpha = std::atan(
          Cy * v_ruddermaindata[i].max_varphi /
          (1 - 0.02 * Cy * std::pow(v_ruddermaindata[i].max_varphi, 2)));
      v_ruddermaindata[i].min_alpha = std::atan(
          Cy * v_ruddermaindata[i].min_varphi /
          (1 - 0.02 * Cy * std::pow(v_ruddermaindata[i].min_varphi, 2)));
    }

    for (int i = 0; i != num_twinfixed; ++i) {
      int index_twinfixed = num_tunnel + num_azimuth + num_mainrudder + i;
      lx(index_twinfixed) = v_twinfixeddata[i].lx;
      ly(index_twinfixed) = v_twinfixeddata[i].ly;
      Omega(index_twinfixed, index_twinfixed) = 50;
      // re-calculate the max thrust of twin fixed thruster
      v_twinfixeddata[i].max_thrust_positive =
          v_twinfixeddata[i].K_positive *
          std::pow(v_twinfixeddata[i].max_rotation, 2);
      v_twinfixeddata[i].max_thrust_negative =
          v_twinfixeddata[i].K_negative *
          std::pow(v_twinfixeddata[i].max_rotation, 2);
    }
    // quadratic penality matrix for error
    setQ(CONTROLMODE::MANUAL);

    initializemosekvariables();

    initializeMosekAPI();
  }

  void initializemosekvariables() {
    int _mdouble = 2 * m;
    int _mquintuple = 6 * m;
    // assign value to the bounds on variables
    for (int i = 0; i != _mdouble; ++i) {
      bkx[i] = MSK_BK_RA;
      blx[i] = 0.0;
      bux[i] = 0.0;
    }
    for (int j = 0; j != n; ++j) {
      int index_n = j + _mdouble;
      bkx[index_n] = MSK_BK_FR;
      blx[index_n] = -MSK_INFINITY;
      bux[index_n] = +MSK_INFINITY;
    }

    // assign value to the linear constraints
    for (int i = 0; i != _mdouble; ++i) {
      int _itriple = 3 * i;
      aptrb[i] = _itriple;
      aptre[i] = _itriple + 3;

      for (int j = 0; j != 3; ++j) {
        asub[_itriple + j] = j;
        aval[_itriple + j] = 0.0;
      }
    }
    for (int j = 0; j != n; ++j) {
      aptrb[j + _mdouble] = _mquintuple + j;
      aptre[j + _mdouble] = _mquintuple + j + 1;
      asub[j + _mquintuple] = j;
      aval[j + _mquintuple] = 1.0;
      bkc[j] = MSK_BK_FX;
      blc[j] = 0;
      buc[j] = 0;
    }

    // assign value to the objective
    for (int i = 0; i != numvar; ++i) {
      qsubi[i] = i;
      qsubj[i] = i;
      g[i] = 0;
    }
    for (int i = 0; i != m; ++i) {
      qval[i] = 0;
    }
    for (int i = 0; i != m; ++i) {
      qval[i + m] = Omega(i, i);
    }
    for (int j = 0; j != n; ++j) {
      qval[j + 2 * m] = Q(j, j);
    }
  }  // initializemosekvariables

  void initializeMosekAPI() {
    /* Create the mosek environment. */
    r = MSK_makeenv(&env, NULL);
    /* Create the optimization task. */
    r = MSK_maketask(env, num_constraints, numvar, &task);
    // set up the threads used by mosek
    const int QP_THREADS_USED = 1;  // # of thread used by mosek
    r = MSK_putintparam(task, MSK_IPAR_NUM_THREADS, QP_THREADS_USED);
    // r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
    // r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, NULL);
    // append num_constrainsts empty contraints
    r = MSK_appendcons(task, num_constraints);
    // append numvar emtpy variables
    r = MSK_appendvars(task, numvar);
  }

  // calculate the contraints of tunnel thruster
  // depend on the desired force in the Y direction or Mz direction
  void calculateconstraints_tunnel(const controllerRTdata<m, n> &_RTdata,
                                   double _desired_Mz) {
    for (int i = 0; i != num_tunnel; ++i) {
      int _maxdeltar = v_tunnelthrusterdata[i].max_delta_rotation;
      double _Kp = v_tunnelthrusterdata[i].K_positive;
      double _Kn = v_tunnelthrusterdata[i].K_negative;
      if (0 < _RTdata.feedback_rotation(i) &&
          _RTdata.feedback_rotation(i) <= _maxdeltar) {
        // specify the first case
        if (_desired_Mz > 0) {
          upper_delta_alpha(i) = 0;
          lower_delta_alpha(i) = 0;
          lower_delta_u(i) = -_RTdata.feedback_u(i);
          upper_delta_u(i) =
              _Kp * std::pow(_RTdata.feedback_rotation(i) + _maxdeltar, 2) -
              _RTdata.feedback_u(i);
        } else {
          upper_delta_alpha(i) = -M_PI;
          lower_delta_alpha(i) = -M_PI;
          lower_delta_u(i) = -_RTdata.feedback_u(i);
          upper_delta_u(i) =
              _Kn * std::pow(_RTdata.feedback_rotation(i) - _maxdeltar, 2) -
              _RTdata.feedback_u(i);
        }
      } else if (-_maxdeltar <= _RTdata.feedback_rotation(i) &&
                 _RTdata.feedback_rotation(i) < 0) {
        if (_desired_Mz > 0) {
          // specify the second case
          upper_delta_alpha(i) = M_PI;
          lower_delta_alpha(i) = M_PI;
          lower_delta_u(i) = -_RTdata.feedback_u(i);
          upper_delta_u(i) =
              _Kp * std::pow(_RTdata.feedback_rotation(i) + _maxdeltar, 2) -
              _RTdata.feedback_u(i);
        } else {
          // specify the first case
          upper_delta_alpha(i) = 0;
          lower_delta_alpha(i) = 0;
          lower_delta_u(i) = -_RTdata.feedback_u(i);
          upper_delta_u(i) =
              _Kn * std::pow(_RTdata.feedback_rotation(i) - _maxdeltar, 2) -
              _RTdata.feedback_u(i);
        }

      } else if (_RTdata.feedback_rotation(i) > _maxdeltar) {
        lower_delta_alpha(i) = 0;
        upper_delta_alpha(i) = 0;
        upper_delta_u(i) = std::min(
            v_tunnelthrusterdata[i].max_thrust_positive - _RTdata.feedback_u(i),
            _Kp * std::pow(_RTdata.feedback_rotation(i) + _maxdeltar, 2) -
                _RTdata.feedback_u(i));
        lower_delta_u(i) =
            _Kp * std::pow(_RTdata.feedback_rotation(i) - _maxdeltar, 2) -
            _RTdata.feedback_u(i);

      } else {
        lower_delta_alpha(i) = 0;
        upper_delta_alpha(i) = 0;
        upper_delta_u(i) = std::min(
            _Kn * std::pow(_RTdata.feedback_rotation(i) - _maxdeltar, 2) -
                _RTdata.feedback_u(i),
            v_tunnelthrusterdata[i].max_thrust_negative -
                _RTdata.feedback_u(i));
        lower_delta_u(i) =
            _Kn * std::pow(_RTdata.feedback_rotation(i) + _maxdeltar, 2) -
            _RTdata.feedback_u(i);
      }
    }
  }  // calculateconstraints_tunnel

  // calculate the consraints of azimuth thruster
  void calculateconstraints_azimuth(const controllerRTdata<m, n> &_RTdata) {
    for (int j = 0; j != num_azimuth; ++j) {
      int index_azimuth = j + num_tunnel;
      /* contraints on the increment of angle */
      upper_delta_alpha(index_azimuth) =
          std::min(v_azimuththrusterdata[j].max_delta_alpha,
                   v_azimuththrusterdata[j].max_alpha -
                       _RTdata.feedback_alpha(index_azimuth));
      lower_delta_alpha(index_azimuth) =
          std::max(-v_azimuththrusterdata[j].max_delta_alpha,
                   v_azimuththrusterdata[j].min_alpha -
                       _RTdata.feedback_alpha(index_azimuth));
      /* contraints on the increment of thrust */
      double K = v_azimuththrusterdata[j].K;
      int _maxdeltar = v_azimuththrusterdata[j].max_delta_rotation;
      upper_delta_u(index_azimuth) =
          std::min(v_azimuththrusterdata[j].max_thrust,
                   K * std::pow(_RTdata.feedback_rotation(index_azimuth) +
                                    _maxdeltar,
                                2)) -
          _RTdata.feedback_u(index_azimuth);

      if (_RTdata.feedback_rotation(index_azimuth) < _maxdeltar)
        lower_delta_u(index_azimuth) = v_azimuththrusterdata[j].min_thrust -
                                       _RTdata.feedback_u(index_azimuth);
      else
        lower_delta_u(index_azimuth) =
            std::max(K * std::pow(_RTdata.feedback_rotation(index_azimuth) -
                                      _maxdeltar,
                                  2),
                     v_azimuththrusterdata[j].min_thrust) -
            _RTdata.feedback_u(index_azimuth);
    }
  }  // calculateconstraints_azimuth

  //  calculate the consraints of thruster with rudder
  void calculateconstraints_rudder(const controllerRTdata<m, n> &_RTdata) {
    for (int k = 0; k != num_mainrudder; ++k) {
      int index_rudder = k + num_tunnel + num_azimuth;
      double K = v_ruddermaindata[k].K;
      int _maxdeltar = v_ruddermaindata[k].max_delta_rotation;
      double Cy = v_ruddermaindata[k].Cy;
      /* contraints on the rudder angle */
      double rudderangle =
          static_cast<double>(_RTdata.feedback_alpha_deg(index_rudder));
      double rudderangle_upper =
          std::min(rudderangle + v_ruddermaindata[k].max_delta_varphi,
                   v_ruddermaindata[k].max_varphi);
      double rudderangle_lower =
          std::max(rudderangle - v_ruddermaindata[k].max_delta_varphi,
                   v_ruddermaindata[k].min_varphi);

      /* contraints on the increment of alpha */
      upper_delta_alpha(index_rudder) =
          std::atan(Cy * rudderangle_upper /
                    (1 - 0.02 * Cy * std::pow(rudderangle_upper, 2))) -
          _RTdata.feedback_alpha(index_rudder);
      lower_delta_alpha(index_rudder) =
          std::atan(Cy * rudderangle_lower /
                    (1 - 0.02 * Cy * std::pow(rudderangle_lower, 2))) -
          _RTdata.feedback_alpha(index_rudder);
      /* contraints on the increment of thrust */
      // max and min of effective thrust
      double _max_u = std::min(
          v_ruddermaindata[k].max_thrust,
          K * std::pow(_RTdata.feedback_rotation(index_rudder) + _maxdeltar,
                       2));

      double _min_u = 0;
      if (_RTdata.feedback_rotation(index_rudder) < _maxdeltar)
        _min_u = v_ruddermaindata[k].min_thrust;
      else
        _min_u = std::max(
            K * std::pow(_RTdata.feedback_rotation(index_rudder) - _maxdeltar,
                         2),
            v_ruddermaindata[k].min_thrust);

      // max and min of sqrt root
      double max_squrevarphi = 0;
      double min_squrevarphi = 0;
      if (rudderangle_upper > 0 && rudderangle_lower > 0) {
        max_squrevarphi = std::pow(rudderangle_upper, 2);
        min_squrevarphi = std::pow(rudderangle_lower, 2);

      } else if (rudderangle_upper < 0 && rudderangle_lower < 0) {
        max_squrevarphi = std::pow(rudderangle_lower, 2);
        min_squrevarphi = std::pow(rudderangle_upper, 2);
      } else {
        max_squrevarphi = std::max(std::pow(rudderangle_lower, 2),
                                   std::pow(rudderangle_upper, 2));
        min_squrevarphi = 0;
      }

      double _max_usqrtterm = 0;
      double _min_usqrtterm = 0;

      double _a = 0.0004 * std::pow(Cy, 2);
      double _b = std::pow(Cy, 2) - 0.04 * Cy;
      double _c = 1.0;
      double min_point_x = 100.0 / Cy - 2500.0;
      if (min_squrevarphi > min_point_x) {
        _max_usqrtterm =
            std::sqrt(computeabcvalue(_a, _b, _c, max_squrevarphi));
        _min_usqrtterm =
            std::sqrt(computeabcvalue(_a, _b, _c, min_squrevarphi));
      } else if (max_squrevarphi < min_point_x) {
        _max_usqrtterm =
            std::sqrt(computeabcvalue(_a, _b, _c, min_squrevarphi));
        _min_usqrtterm =
            std::sqrt(computeabcvalue(_a, _b, _c, max_squrevarphi));
      } else {
        _max_usqrtterm =
            std::sqrt(std::max(computeabcvalue(_a, _b, _c, min_squrevarphi),
                               computeabcvalue(_a, _b, _c, max_squrevarphi)));
        _min_usqrtterm = std::sqrt(computeabcvalue(_a, _b, _c, min_point_x));
      }
      upper_delta_u(index_rudder) =
          _max_u * _max_usqrtterm - _RTdata.feedback_u(index_rudder);

      lower_delta_u(index_rudder) =
          _min_u * _min_usqrtterm - _RTdata.feedback_u(index_rudder);
    }
  }  // calculateconstraints_rudder

  // calculate the contraints of twin fixed thruster
  // depend on the desired force in the X direction and Mz direction
  void calculateconstraints_twinfixed(const controllerRTdata<m, n> &_RTdata,
                                      double _desired_Fx, double _desired_Mz) {
    // compute the desired thrust of each thruster
    Eigen::Vector2d _desired_tau = Eigen::Vector2d::Zero();

    double delta_l = 1.0 / (v_twinfixeddata[1].ly - v_twinfixeddata[0].ly);
    _desired_tau(0) =
        (_desired_Mz + _desired_Fx * v_twinfixeddata[1].ly) * delta_l;
    _desired_tau(1) =
        -(_desired_Mz + _desired_Fx * v_twinfixeddata[0].ly) * delta_l;

    for (int i = 0; i != num_twinfixed; ++i) {
      int index_tf = i + num_tunnel + num_azimuth + num_mainrudder;
      int _maxdn = v_twinfixeddata[i].max_delta_rotation;
      int _maxdnp2n = v_twinfixeddata[i].max_delta_rotation_p2n;
      double _Kp = v_twinfixeddata[i].K_positive;
      double _Kn = v_twinfixeddata[i].K_negative;
      int _n0 = _RTdata.feedback_rotation(index_tf);
      double _u0 = _RTdata.feedback_u(index_tf);

      if (_n0 >= _maxdn) {
        upper_delta_alpha(index_tf) = 0;
        lower_delta_alpha(index_tf) = 0;
        upper_delta_u(index_tf) =
            std::min(v_twinfixeddata[i].max_thrust_positive - _u0,
                     _Kp * std::pow(_n0 + _maxdn, 2) - _u0);
        lower_delta_u(index_tf) = _Kp * std::pow(_n0 - _maxdn, 2) - _u0;
      } else if (_maxdnp2n <= _n0 && _n0 < _maxdn) {
        upper_delta_alpha(index_tf) = 0;
        lower_delta_alpha(index_tf) = 0;
        upper_delta_u(index_tf) = _Kp * std::pow(_n0 + _maxdnp2n, 2) - _u0;
        lower_delta_u(index_tf) = _Kp * std::pow(_n0 - _maxdnp2n, 2) - _u0;
      } else if (0 < _n0 && _n0 < _maxdnp2n) {
        if (_desired_tau(i) < 0) {
          // change the direction
          upper_delta_alpha(index_tf) = M_PI;
          lower_delta_alpha(index_tf) = M_PI;
          lower_delta_u(index_tf) = _Kn * std::pow(_n0 - _maxdnp2n, 2) - _u0;
          upper_delta_u(index_tf) = lower_delta_u(index_tf);
        } else {
          upper_delta_alpha(index_tf) = 0;
          lower_delta_alpha(index_tf) = 0;
          lower_delta_u(index_tf) = _Kp * std::pow(_n0 + _maxdnp2n, 2) - _u0;
          upper_delta_u(index_tf) = lower_delta_u(index_tf);
        }

      } else if (-_maxdnp2n < _n0 && _n0 < 0) {
        if (_desired_tau(i) < 0) {
          upper_delta_alpha(index_tf) = 0;
          lower_delta_alpha(index_tf) = 0;
          lower_delta_u(index_tf) = _Kn * std::pow(_n0 - _maxdnp2n, 2) - _u0;
          upper_delta_u(index_tf) = lower_delta_u(index_tf);
        } else {
          // change the direction
          upper_delta_alpha(index_tf) = -M_PI;
          lower_delta_alpha(index_tf) = -M_PI;
          lower_delta_u(index_tf) = _Kp * std::pow(_n0 + _maxdnp2n, 2) - _u0;
          upper_delta_u(index_tf) = lower_delta_u(index_tf);
        }

      } else if (-_maxdn < _n0 && _n0 <= -_maxdnp2n) {
        upper_delta_alpha(index_tf) = 0;
        lower_delta_alpha(index_tf) = 0;
        upper_delta_u(index_tf) = _Kn * std::pow(_n0 - _maxdnp2n, 2) - _u0;
        lower_delta_u(index_tf) = _Kn * std::pow(_n0 + _maxdnp2n, 2) - _u0;
      } else {
        upper_delta_alpha(index_tf) = 0;
        lower_delta_alpha(index_tf) = 0;
        upper_delta_u(index_tf) =
            std::min(v_twinfixeddata[i].max_thrust_negative - _u0,
                     _Kn * std::pow(_n0 - _maxdn, 2) - _u0);
        lower_delta_u(index_tf) = _Kn * std::pow(_n0 + _maxdn, 2) - _u0;
      }
    }

  }  // calculateconstraints_twinfixed

  // calculate based on the feedback rotation and alpha_deg
  void update_formerstep_feedback(controllerRTdata<m, n> &_RTdata) {
    // convert the double alpha(rad) to int alpha(deg)
    convert_alpha_int2radian(_RTdata.feedback_alpha_deg,
                             _RTdata.feedback_alpha);
    // update u
    calculateu(_RTdata);
  }

  // calculate the command at the next time step
  void update_nextstep_command(controllerRTdata<m, n> &_RTdata) {
    // calculate delta variable using Mosek results
    delta_u = results.head(m);
    delta_alpha = results.segment(m, m);
    // update alpha and u
    updateAlphaandU(_RTdata.feedback_u, _RTdata.feedback_alpha,
                    _RTdata.command_u, _RTdata.command_alpha);
    // convert the double alpha(rad) to int alpha(deg)
    convert_alpha_radian2int(_RTdata.command_alpha, _RTdata.command_alpha_deg);
    // update rotation speed
    calculaterotation(_RTdata);
    // // update BalphaU
    // _RTdata.BalphaU = calculateBalphau(_RTdata.alpha, _RTdata.u);
  }

  // update alpha and u using computed delta_alpha and delta_u (command)
  void updateAlphaandU(const vectormd &_feedback_u,
                       const vectormd &_feedback_alpha, vectormd &_command_u,
                       vectormd &_command_alpha) {
    _command_u = _feedback_u + delta_u;
    _command_alpha = _feedback_alpha + delta_alpha;
  }

  // convert the radian to deg, and round to integer (command)
  void convert_alpha_radian2int(const vectormd &_alpha, vectormi &_alpha_deg) {
    // round to int (deg) for tunnel and azimuth thrusters
    for (int i = 0; i != (num_tunnel + num_azimuth); ++i)
      _alpha_deg(i) = rad2degree(_alpha(i));

    // convert alpha to varphi (rudder angle) for thrusters with rudder
    for (int k = 0; k != num_mainrudder; ++k) {
      int r_index = num_tunnel + num_azimuth + k;

      if (rad2degree(_alpha(r_index)) == 0) {
        _alpha_deg(r_index) = 0;
        continue;
      }

      double cytan = v_ruddermaindata[k].Cy / std::tan(_alpha(r_index));
      double sqrtterm =
          std::sqrt(std::pow(cytan, 2) + 0.08 * v_ruddermaindata[k].Cy);
      double varphi = 0;
      if (_alpha(r_index) > 0)
        varphi = 25 * (sqrtterm - cytan) / v_ruddermaindata[k].Cy;
      else
        varphi = 25 * (-sqrtterm - cytan) / v_ruddermaindata[k].Cy;
      _alpha_deg(r_index) = static_cast<int>(std::round(varphi));
    }

    // round to int (deg) for twin fixed thruster
    for (int l = 0; l != num_twinfixed; ++l) {
      int t_index = num_tunnel + num_azimuth + num_mainrudder + l;
      _alpha_deg(t_index) = rad2degree(_alpha(t_index));
    }
  }  // convert_alpha_radian2int

  // convert the deg to radian (feedback)
  void convert_alpha_int2radian(const vectormi &_alpha_deg, vectormd &_alpha) {
    // tunnel and azimuth thrusters
    for (int i = 0; i != (num_tunnel + num_azimuth); ++i)
      _alpha(i) = degree2rad(_alpha_deg(i));

    // convert alpha to varphi (rudder angle) for thrusters with rudder
    for (int k = 0; k != num_mainrudder; ++k) {
      int r_index = num_tunnel + num_azimuth + k;

      double cytan = v_ruddermaindata[k].Cy * _alpha_deg(r_index) /
                     (1 - 0.02 * v_ruddermaindata[k].Cy * _alpha_deg(r_index) *
                              _alpha_deg(r_index));

      _alpha(r_index) = std::atan(cytan);
    }

    // round to int (deg) for twin fixed thruster
    for (int l = 0; l != num_twinfixed; ++l) {
      int t_index = num_tunnel + num_azimuth + num_mainrudder + l;
      _alpha(t_index) = degree2rad(_alpha_deg(t_index));
    }
  }  // convert_alpha_int2radian
  // calcuate rotation speed of each thruster based on thrust (command)
  void calculaterotation(controllerRTdata<m, n> &_RTdata) {
    // tunnel thruster
    for (int i = 0; i != num_tunnel; ++i) {
      int t_rotation = 0;
      if (_RTdata.command_alpha(i) < 0) {
        t_rotation = static_cast<int>(std::sqrt(
            abs(_RTdata.command_u(i)) / v_tunnelthrusterdata[i].K_negative));
        if (t_rotation == 0) {
          _RTdata.command_rotation(i) = -1;  // prevent zero
          _RTdata.command_u(i) = v_tunnelthrusterdata[i].K_negative;
        } else
          _RTdata.command_rotation(i) = -t_rotation;

      } else {
        t_rotation = static_cast<int>(std::sqrt(
            abs(_RTdata.command_u(i)) / v_tunnelthrusterdata[i].K_positive));

        if (t_rotation == 0) {
          _RTdata.command_rotation(i) = 1;  // prevent zero
          _RTdata.command_u(i) = v_tunnelthrusterdata[i].K_positive;
        } else
          _RTdata.command_rotation(i) = t_rotation;
      }
    }

    // azimuth thruster
    for (int j = 0; j != num_azimuth; ++j) {
      int index_azimuth = j + num_tunnel;

      int t_rotation = static_cast<int>(sqrt(
          abs(_RTdata.command_u(index_azimuth)) / v_azimuththrusterdata[j].K));
      if (t_rotation < v_azimuththrusterdata[j].min_rotation) {
        _RTdata.command_rotation(index_azimuth) =
            v_azimuththrusterdata[j].min_rotation;
        _RTdata.command_u(index_azimuth) = v_azimuththrusterdata[j].min_thrust;
      } else
        _RTdata.command_rotation(index_azimuth) = t_rotation;
    }

    // thruster with rudder
    for (int k = 0; k != num_mainrudder; ++k) {
      int index_rudder = k + num_tunnel + num_azimuth;
      double Cy = v_ruddermaindata[k].Cy;
      double _a = 0.0004 * std::pow(Cy, 2);
      double _b = std::pow(Cy, 2) - 0.04 * Cy;
      double _c = 1.0;

      double sqrtrootterm = std::sqrt(computeabcvalue(
          _a, _b, _c, std::pow(_RTdata.command_alpha_deg(index_rudder), 2)));

      int t_rotation =
          static_cast<int>(sqrt(abs(_RTdata.command_u(index_rudder)) /
                                (sqrtrootterm * v_ruddermaindata[k].K)));
      if (t_rotation < v_ruddermaindata[k].min_rotation) {
        _RTdata.command_rotation(index_rudder) =
            v_ruddermaindata[k].min_rotation;
        _RTdata.command_u(index_rudder) = v_ruddermaindata[k].min_thrust;
      } else
        _RTdata.command_rotation(index_rudder) = t_rotation;
    }

    // twin fixed thruster
    for (int l = 0; l != num_twinfixed; ++l) {
      int index_tk = l + num_tunnel + num_azimuth + num_mainrudder;
      int t_rotation = 0;
      if (_RTdata.command_alpha(index_tk) > 0.5 * M_PI) {
        t_rotation = static_cast<int>(std::sqrt(
            abs(_RTdata.command_u(index_tk)) / v_twinfixeddata[l].K_negative));

        if (t_rotation == 0) {
          _RTdata.command_rotation(index_tk) = -1;  // prevent zero
          _RTdata.command_u(index_tk) = v_twinfixeddata[l].K_negative;
        } else
          _RTdata.command_rotation(index_tk) = -t_rotation;

      } else {
        t_rotation = static_cast<int>(std::sqrt(
            abs(_RTdata.command_u(index_tk)) / v_twinfixeddata[l].K_positive));

        if (t_rotation == 0) {
          _RTdata.command_rotation(index_tk) = 1;  // prevent zero
          _RTdata.command_u(index_tk) = v_twinfixeddata[l].K_positive;
        } else
          _RTdata.command_rotation(index_tk) = t_rotation;
      }
    }
  }  // calculaterotation

  // calcuate thrust based on rotation speed of each thruster (feedback)
  void calculateu(controllerRTdata<m, n> &_RTdata) {
    // tunnel thruster
    for (int i = 0; i != num_tunnel; ++i) {
      if (_RTdata.feedback_rotation(i) < 0)
        _RTdata.feedback_u(i) = v_tunnelthrusterdata[i].K_negative *
                                _RTdata.feedback_rotation(i) *
                                _RTdata.feedback_rotation(i);
      else if (_RTdata.feedback_rotation(i) > 0)
        _RTdata.feedback_u(i) = v_tunnelthrusterdata[i].K_positive *
                                _RTdata.feedback_rotation(i) *
                                _RTdata.feedback_rotation(i);
      else {
        _RTdata.feedback_rotation(i) = 1;
        _RTdata.feedback_u(i) = v_tunnelthrusterdata[i].K_positive;
      }
    }

    // azimuth thruster
    for (int j = 0; j != num_azimuth; ++j) {
      int index_azimuth = j + num_tunnel;

      if (_RTdata.feedback_rotation(index_azimuth) <
          v_azimuththrusterdata[j].min_rotation) {
        _RTdata.feedback_rotation(index_azimuth) =
            v_azimuththrusterdata[j].min_rotation;
        _RTdata.feedback_u(index_azimuth) = v_azimuththrusterdata[j].min_thrust;
      } else {
        _RTdata.feedback_u(index_azimuth) =
            v_azimuththrusterdata[j].K *
            _RTdata.feedback_rotation(index_azimuth) *
            _RTdata.feedback_rotation(index_azimuth);
      }
    }

    // thruster with rudder
    for (int k = 0; k != num_mainrudder; ++k) {
      int index_rudder = k + num_tunnel + num_azimuth;
      double Cy = v_ruddermaindata[k].Cy;
      double _a = 0.0004 * std::pow(Cy, 2);
      double _b = std::pow(Cy, 2) - 0.04 * Cy;
      double _c = 1.0;

      double sqrtrootterm = std::sqrt(computeabcvalue(
          _a, _b, _c, std::pow(_RTdata.feedback_alpha_deg(index_rudder), 2)));

      if (_RTdata.feedback_rotation(index_rudder) <
          v_ruddermaindata[k].min_rotation) {
        _RTdata.feedback_rotation(index_rudder) =
            v_ruddermaindata[k].min_rotation;
        _RTdata.feedback_u(index_rudder) = v_ruddermaindata[k].min_thrust;
      } else
        _RTdata.feedback_u(index_rudder) =
            sqrtrootterm * v_ruddermaindata[k].K *
            _RTdata.feedback_rotation(index_rudder) *
            _RTdata.feedback_rotation(index_rudder);
    }

    // twin fixed thruster
    for (int l = 0; l != num_twinfixed; ++l) {
      int index_tk = l + num_tunnel + num_azimuth + num_mainrudder;

      if (_RTdata.feedback_rotation(index_tk) < 0)
        _RTdata.feedback_u(index_tk) = v_twinfixeddata[l].K_negative *
                                       _RTdata.feedback_rotation(index_tk) *
                                       _RTdata.feedback_rotation(index_tk);
      else if (_RTdata.feedback_rotation(index_tk) > 0)
        _RTdata.feedback_u(index_tk) = v_twinfixeddata[l].K_positive *
                                       _RTdata.feedback_rotation(index_tk) *
                                       _RTdata.feedback_rotation(index_tk);
      else {
        _RTdata.feedback_rotation(index_tk) = 1;
        _RTdata.feedback_u(index_tk) = v_twinfixeddata[l].K_positive;
      }
    }
  }  // calculateu

  // calculate Balpha as function of alpha
  matrixnmd calculateBalpha(const vectormd &t_alpha) {
    matrixnmd _B_alpha = matrixnmd::Zero();
    double angle = 0;
    double t_cos = 0;
    double t_sin = 0;
    for (int i = 0; i != m; ++i) {
      angle = t_alpha(i);
      t_cos = cos(angle);
      t_sin = sin(angle);
      _B_alpha(0, i) = t_cos;
      _B_alpha(1, i) = t_sin;
      _B_alpha(2, i) = -ly(i) * t_cos + lx(i) * t_sin;
    }
    return _B_alpha;
  }  // calculateBalpha

  // calculate the rho term in thruster allocation
  double calculateRhoTerm(const vectormd &t_alpha, double epsilon = 0.1,
                          double rho = 10) {
    auto _B_alpha = calculateBalpha(t_alpha);
    matrixnnd BBT = _B_alpha * _B_alpha.transpose();
    return rho / (epsilon + BBT.determinant());
  }  // calculateRhoTerm

  // calculate Jacobian using central difference
  void calculateJocobianRhoTerm(const vectormd &t_alpha) {
    for (int i = 0; i != m; ++i) {
      auto alpha_plus = t_alpha;
      auto alpha_minus = t_alpha;
      alpha_plus(i) += derivative_dx;
      alpha_minus(i) -= derivative_dx;
      d_rho(i) =
          (calculateRhoTerm(alpha_plus) - calculateRhoTerm(alpha_minus)) /
          (2 * derivative_dx);
    }
  }  // calculateJocobianRhoTerm

  // calculate the Balpha u term
  vectornd calculateBalphau(const vectormd &t_alpha, const vectormd &t_u) {
    return calculateBalpha(t_alpha) * t_u;
  }  // calculateBalphau
  // calculate the Balpha u term
  vectornd calculateBalphau(const matrixnmd &t_B_alpha, const vectormd &t_u) {
    return t_B_alpha * t_u;

  }  // calculateBalphau

  // calculate derivative of Balpha times u
  void calculateJocobianBalphaU(const vectormd &t_alpha, const vectormd &t_u) {
    for (int i = 0; i != m; ++i) {
      auto alpha_plus = t_alpha;
      auto alpha_minus = t_alpha;
      alpha_plus(i) += derivative_dx;
      alpha_minus(i) -= derivative_dx;
      d_Balpha_u.col(i) = (calculateBalphau(alpha_plus, t_u) -
                           calculateBalphau(alpha_minus, t_u)) /
                          (2 * derivative_dx);
    }
  }  // calculateJocobianBalphaU

  // calculate g_deltau and Q_deltau
  void calculateDeltauQ(const vectormd &t_u) {
    vectormd d_utemp = vectormd::Zero();
    d_utemp = t_u.cwiseSqrt();
    g_deltau = 1.5 * d_utemp;
    vectormd Q_temp = vectormd::Zero();
    // Q_temp = 0.75 * d_utemp.cwiseInverse();
    Q_temp = 7.5 * d_utemp.cwiseInverse();
    Q_deltau = Q_temp.asDiagonal();
  }  // calculateDeltauQ

  // calculate the BalphaU and b
  void calculateb(const vectornd &_tau, const vectornd &_BalphaU) {
    b = _tau - _BalphaU;
  }

  // update parameters in thruster allocation for each time step
  void updateTAparameters(controllerRTdata<m, n> &_RTdata) {
    B_alpha = calculateBalpha(_RTdata.feedback_alpha);
    // update BalphaU
    _RTdata.BalphaU = calculateBalphau(B_alpha, _RTdata.feedback_u);

    if constexpr (index_actuation == ACTUATION::FULLYACTUATED)
      calculateJocobianRhoTerm(_RTdata.feedback_alpha);
    calculateJocobianBalphaU(_RTdata.feedback_alpha, _RTdata.feedback_u);
    calculateDeltauQ(_RTdata.feedback_u);
    calculateb(_RTdata.tau, _RTdata.BalphaU);
    calculateconstraints_tunnel(_RTdata, _RTdata.tau(2));
    calculateconstraints_azimuth(_RTdata);
    calculateconstraints_rudder(_RTdata);
    if (num_twinfixed > 0)
      calculateconstraints_twinfixed(_RTdata, _RTdata.tau(0), _RTdata.tau(2));
  }
  // update parameters in QP for each time step
  void updateMosekparameters() {
    // update A values
    for (int i = 0; i != m; ++i) {
      for (int j = 0; j != n; ++j) {
        aval[n * i + j] = B_alpha(j, i);
        aval[n * (i + m) + j] = d_Balpha_u(j, i);
      }
    }

    // update linear constraints
    for (int i = 0; i != num_constraints; ++i) {
      blc[i] = b(i);
      buc[i] = b(i);
    }

    for (int i = 0; i != m; ++i) {
      // update variable constraints
      blx[i] = lower_delta_u(i);
      bux[i] = upper_delta_u(i);
      blx[m + i] = lower_delta_alpha(i);
      bux[m + i] = upper_delta_alpha(i);
      // update objective g and Q
      g[i] = g_deltau(i);
      g[i + m] = d_rho(i);
      qval[i] = Q_deltau(i, i);
    }
  }  // updateMosekparameters

  // 一元二次方程
  double computeabcvalue(double a, double b, double c, double x) {
    return a * x * x + b * x + c;
  }

  // convert rad to degree
  int rad2degree(double _rad) { return std::round(_rad * 180 / M_PI); }
  // convert degree to rad
  double degree2rad(int _degree) { return _degree * M_PI / 180.0; }

  // solve QP using Mosek solver
  void onestepmosek() {
    MSKint32t i, j;
    double t_results[2 * m + n];
    results.setZero();
    if (r == MSK_RES_OK) {
      for (j = 0; j < numvar; ++j) {
        /* Set the linear term g_j in the objective.*/
        r = MSK_putcj(task, j, g[j]);

        /* Set the bounds on variable j.
         blx[j] <= x_j <= bux[j] */
        r = MSK_putvarbound(task, j, /* Index of variable.*/
                            bkx[j],  /* Bound key.*/
                            blx[j],  /* Numerical value of lower bound.*/
                            bux[j]); /* Numerical value of upper bound.*/

        /* Input column j of A */
        r = MSK_putacol(
            task, j,             /* Variable (column) index.*/
            aptre[j] - aptrb[j], /* Number of non-zeros in column j.*/
            asub + aptrb[j],     /* Pointer to row indexes of column j.*/
            aval + aptrb[j]);    /* Pointer to Values of column j.*/
      }

      /* Set the bounds on constraints.
         for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
      for (i = 0; i < num_constraints; ++i)
        r = MSK_putconbound(task, i, /* Index of constraint.*/
                            bkc[i],  /* Bound key.*/
                            blc[i],  /* Numerical value of lower bound.*/
                            buc[i]); /* Numerical value of upper bound.*/

      /* Input the Q for the objective. */
      r = MSK_putqobj(task, numvar, qsubi, qsubj, qval);

      if (r == MSK_RES_OK) {
        MSKrescodee trmcode;

        /* Run optimizer */
        r = MSK_optimizetrm(task, &trmcode);

        /* Print a summary containing information
           about the solution for debugging purposes*/
        // MSK_solutionsummary(task, MSK_STREAM_MSG);

        if (r == MSK_RES_OK) {
          MSKsolstae solsta;
          MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

          switch (solsta) {
            case MSK_SOL_STA_OPTIMAL: {
              /* Request the interior solution. */
              MSK_getxx(task, MSK_SOL_ITR, t_results);
              for (int k = 0; k != numvar; ++k) results(k) = t_results[k];
              break;
            }

            case MSK_SOL_STA_DUAL_INFEAS_CER:
            case MSK_SOL_STA_PRIM_INFEAS_CER: {
              //   CLOG(ERROR, "mosek")
              //       << "Primal or dual infeasibility certificate found.";
              break;
            }
            case MSK_SOL_STA_UNKNOWN: {
              //   CLOG(ERROR, "mosek") << "The status of the solution could not
              //   be "
              //                           "determined.";
              break;
            }
            default: {
              //   CLOG(ERROR, "mosek") << "Other solution status.";
              break;
            }
          }
        } else {
          //   CLOG(ERROR, "mosek") << "Error while optimizing.";
        }
      }
      if (r != MSK_RES_OK) {
        /* In case of an error print error code and description. */
        char symname[MSK_MAX_STR_LEN];
        char desc[MSK_MAX_STR_LEN];
        MSK_getcodedesc(r, symname, desc);

        // CLOG(ERROR, "mosek")
        //     << "An error occurred while optimizing. " << std::string(symname)
        //     << " - " << std::string(desc);
      }
    }
  }  // onestepmosek
};   // end class thrustallocation

#endif /* _THRUSTALLOCATION_H_*/