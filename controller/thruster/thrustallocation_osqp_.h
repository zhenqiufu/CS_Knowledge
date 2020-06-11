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

#include "controllerdata.h"
// #include "easylogging++.h"
#include "osqp.h"

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
        num_constraints(numvar + n),
        A_nnz(2 * (m * n + m + n)),
        index_thrusters(_thrustallocationdata.index_thrusters),
        v_tunnelthrusterdata(_v_tunnelthrusterdata),
        v_azimuththrusterdata(_v_azimuththrusterdata),
        v_ruddermaindata(_v_ruddermaindata),
        v_twinfixeddata(_v_twinfixeddata),
        lx_(vectormd::Zero()),
        ly_(vectormd::Zero()),
        upper_delta_alpha_(vectormd::Zero()),
        lower_delta_alpha_(vectormd::Zero()),
        upper_delta_u_(vectormd::Zero()),
        lower_delta_u_(vectormd::Zero()),
        Q_(matrixnnd::Zero()),
        Omega_(matrixmmd::Zero()),
        Q_deltau_(matrixmmd::Zero()),
        g_deltau_(vectormd::Zero()),
        d_rho_(vectormd::Zero()),
        B_alpha_(matrixnmd::Zero()),
        d_Balpha_u_(matrixnmd::Zero()),
        b_(vectornd::Zero()),
        delta_alpha_(vectormd::Zero()),
        delta_u_(vectormd::Zero()),
        derivative_dx(1e-6),
        results_(Eigen::Matrix<double, 2 * m + n, 1>::Zero()) {
    initializethrusterallocation();
  }

  thrustallocation() = delete;
  ~thrustallocation() {
    // Clean workspace
    osqp_cleanup(osqp_work);

    // Cleanup
    if (osqp_data) {
      if (osqp_data->P) c_free(osqp_data->P);
      if (osqp_data->A) c_free(osqp_data->A);
      c_free(osqp_data);
    }
    if (osqp_settings) c_free(osqp_settings);
  }

  // perform the thrust allocation using QP solver (one step)
  void onestepthrustallocation(controllerRTdata<m, n> &_RTdata) {
    update_formerstep_feedback(_RTdata);
    updateTAparameters(_RTdata);
    updateOSQPparameters();
    onestepOSQP();
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
          Q_(0, 0) = Q_surge;
          Q_(1, 1) = Q_sway;
          Q_(2, 2) = Q_yaw;
          break;
        case CONTROLMODE::HEADINGONLY:
          // empirical value
          Q_(0, 0) = 0.2 * Q_surge;
          Q_(1, 1) = 0.2 * Q_sway;
          Q_(2, 2) = 2 * Q_yaw;
          break;
        case CONTROLMODE::MANEUVERING:
          Q_(0, 0) = Q_surge;
          Q_(1, 1) = 0;  // The penalty for sway error is zero
          Q_(2, 2) = Q_yaw;
          break;
        case CONTROLMODE::DYNAMICPOSITION:
          Q_(0, 0) = Q_surge;
          Q_(1, 1) = Q_sway;
          Q_(2, 2) = Q_yaw;
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
          Q_(0, 0) = Q_surge;
          Q_(1, 1) = 0;  // The penalty for sway error is zero
          Q_(2, 2) = Q_yaw;
          break;
        case CONTROLMODE::HEADINGONLY:
          Q_(0, 0) = 0.2 * Q_surge;
          Q_(1, 1) = 0;  // The penalty for sway error is zero
          Q_(2, 2) = 2 * Q_yaw;
          break;
        case CONTROLMODE::MANEUVERING:
          // Q(0, 0) = 10;  // 取值与螺旋桨最大推力呈负相关
          // // Q(1, 1) = 0;  The penalty for sway error is zero
          // Q(2, 2) = 20;

          // 取值与螺旋桨最大推力呈负相关
          Q_(0, 0) = Q_surge;
          Q_(1, 1) = 0;  // The penalty for sway error is zero
          Q_(2, 2) = Q_yaw;

          break;
        default:
          break;
      }
    }
    // update OSQP
    for (int j = 0; j != n; ++j) {
      osqp_P_x[j + 2 * m] = Q_(j, j);
    }
  }  // setQ

  //
  vectormd lx() const { return lx_; }
  vectormd ly() const { return ly_; }
  vectormd upper_delta_alpha() const { return upper_delta_alpha_; }
  vectormd lower_delta_alpha() const { return lower_delta_alpha_; }
  vectormd upper_delta_u() const { return upper_delta_u_; }
  vectormd lower_delta_u() const { return lower_delta_u_; }
  matrixnnd Q() const { return Q_; }
  matrixmmd Omega() const { return Omega_; }
  matrixmmd Q_deltau() const { return Q_deltau_; }
  vectormd g_deltau() const { return g_deltau_; }
  vectormd d_rho() const { return d_rho_; }
  matrixnmd B_alpha() const { return B_alpha_; }
  matrixnmd d_Balpha_u() const { return d_Balpha_u_; }
  vectornd b() const { return b_; }
  vectormd delta_alpha() const { return delta_alpha_; }
  vectormd delta_u() const { return delta_u_; }
  Eigen::Matrix<double, 2 * m + n, 1> results() const { return results_; }

 private:
  const double Q_surge;
  const double Q_sway;
  const double Q_yaw;

  const int num_tunnel;
  const int num_azimuth;
  const int num_mainrudder;
  const int num_twinfixed;

  const int numvar;           // # of variable in QP
  const int num_constraints;  // # of constraints in QP
  const int A_nnz;            // # of non-zero elements in A in QP

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
  vectormd lx_;
  vectormd ly_;
  // real time constraints of each thruster
  vectormd upper_delta_alpha_;
  vectormd lower_delta_alpha_;
  vectormd upper_delta_u_;
  vectormd lower_delta_u_;
  // quadratic objective
  matrixnnd Q_;
  matrixmmd Omega_;
  matrixmmd Q_deltau_;
  // linear objective
  vectormd g_deltau_;
  vectormd d_rho_;

  // real time constraint matrix in QP (equality constraint)
  matrixnmd B_alpha_;
  matrixnmd d_Balpha_u_;  // Jocobian matrix of Balpha times u
  vectornd b_;

  // real time physical variable in thruster allocation
  vectormd delta_alpha_;  // rad
  vectormd delta_u_;      // N
  // linearized parameters
  double derivative_dx;  // step size of the derivative

  // array to store the optimization results
  Eigen::Matrix<double, 2 * m + n, 1> results_;

  // parameters for OSQP API
  c_float osqp_P_x[2 * m + n];
  c_int osqp_P_i[2 * m + n];
  c_int osqp_P_p[2 * m + n + 1];
  c_float osqp_q[2 * m + n];
  c_float osqp_A_x[2 * (m * n + m + n)];
  c_int osqp_A_i[2 * (m * n + m + n)];
  c_int osqp_A_p[2 * m + n + 1];
  c_float osqp_l[2 * (m + n)];
  c_float osqp_u[2 * (m + n)];

  // OSQP workspace
  c_int osqp_flag;
  OSQPWorkspace *osqp_work;
  OSQPSettings *osqp_settings;
  OSQPData *osqp_data;

  void initializethrusterallocation() {
    assert(num_tunnel + num_azimuth + num_mainrudder + num_twinfixed == m);
    if (num_twinfixed > 0) assert(num_twinfixed == 2);

    for (int i = 0; i != num_tunnel; ++i) {
      lx_(i) = v_tunnelthrusterdata[i].lx;
      ly_(i) = v_tunnelthrusterdata[i].ly;
      Omega_(i, i) = 1;
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
      lx_(index_azimuth) = v_azimuththrusterdata[i].lx;
      ly_(index_azimuth) = v_azimuththrusterdata[i].ly;
      Omega_(index_azimuth, index_azimuth) = 50;
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
      lx_(index_rudder) = v_ruddermaindata[i].lx;
      ly_(index_rudder) = v_ruddermaindata[i].ly;
      Omega_(index_rudder, index_rudder) = 50;
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
      lx_(index_twinfixed) = v_twinfixeddata[i].lx;
      ly_(index_twinfixed) = v_twinfixeddata[i].ly;
      Omega_(index_twinfixed, index_twinfixed) = 50;
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

    initializeOSQPVariables();

    initializeOSQPAPI();
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
          upper_delta_alpha_(i) = 0;
          lower_delta_alpha_(i) = 0;
          lower_delta_u_(i) = -_RTdata.feedback_u(i);
          upper_delta_u_(i) =
              _Kp * std::pow(_RTdata.feedback_rotation(i) + _maxdeltar, 2) -
              _RTdata.feedback_u(i);
        } else {
          upper_delta_alpha_(i) = -M_PI;
          lower_delta_alpha_(i) = -M_PI;
          lower_delta_u_(i) = -_RTdata.feedback_u(i);
          upper_delta_u_(i) =
              _Kn * std::pow(_RTdata.feedback_rotation(i) - _maxdeltar, 2) -
              _RTdata.feedback_u(i);
        }
      } else if (-_maxdeltar <= _RTdata.feedback_rotation(i) &&
                 _RTdata.feedback_rotation(i) < 0) {
        if (_desired_Mz > 0) {
          // specify the second case
          upper_delta_alpha_(i) = M_PI;
          lower_delta_alpha_(i) = M_PI;
          lower_delta_u_(i) = -_RTdata.feedback_u(i);
          upper_delta_u_(i) =
              _Kp * std::pow(_RTdata.feedback_rotation(i) + _maxdeltar, 2) -
              _RTdata.feedback_u(i);
        } else {
          // specify the first case
          upper_delta_alpha_(i) = 0;
          lower_delta_alpha_(i) = 0;
          lower_delta_u_(i) = -_RTdata.feedback_u(i);
          upper_delta_u_(i) =
              _Kn * std::pow(_RTdata.feedback_rotation(i) - _maxdeltar, 2) -
              _RTdata.feedback_u(i);
        }

      } else if (_RTdata.feedback_rotation(i) > _maxdeltar) {
        lower_delta_alpha_(i) = 0;
        upper_delta_alpha_(i) = 0;
        upper_delta_u_(i) = std::min(
            v_tunnelthrusterdata[i].max_thrust_positive - _RTdata.feedback_u(i),
            _Kp * std::pow(_RTdata.feedback_rotation(i) + _maxdeltar, 2) -
                _RTdata.feedback_u(i));
        lower_delta_u_(i) =
            _Kp * std::pow(_RTdata.feedback_rotation(i) - _maxdeltar, 2) -
            _RTdata.feedback_u(i);

      } else {
        lower_delta_alpha_(i) = 0;
        upper_delta_alpha_(i) = 0;
        upper_delta_u_(i) = std::min(
            _Kn * std::pow(_RTdata.feedback_rotation(i) - _maxdeltar, 2) -
                _RTdata.feedback_u(i),
            v_tunnelthrusterdata[i].max_thrust_negative -
                _RTdata.feedback_u(i));
        lower_delta_u_(i) =
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
      upper_delta_alpha_(index_azimuth) =
          std::min(v_azimuththrusterdata[j].max_delta_alpha,
                   v_azimuththrusterdata[j].max_alpha -
                       _RTdata.feedback_alpha(index_azimuth));
      lower_delta_alpha_(index_azimuth) =
          std::max(-v_azimuththrusterdata[j].max_delta_alpha,
                   v_azimuththrusterdata[j].min_alpha -
                       _RTdata.feedback_alpha(index_azimuth));
      /* contraints on the increment of thrust */
      double K = v_azimuththrusterdata[j].K;
      int _maxdeltar = v_azimuththrusterdata[j].max_delta_rotation;
      upper_delta_u_(index_azimuth) =
          std::min(v_azimuththrusterdata[j].max_thrust,
                   K * std::pow(_RTdata.feedback_rotation(index_azimuth) +
                                    _maxdeltar,
                                2)) -
          _RTdata.feedback_u(index_azimuth);

      if (_RTdata.feedback_rotation(index_azimuth) < _maxdeltar)
        lower_delta_u_(index_azimuth) = v_azimuththrusterdata[j].min_thrust -
                                        _RTdata.feedback_u(index_azimuth);
      else
        lower_delta_u_(index_azimuth) =
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
      upper_delta_alpha_(index_rudder) =
          std::atan(Cy * rudderangle_upper /
                    (1 - 0.02 * Cy * std::pow(rudderangle_upper, 2))) -
          _RTdata.feedback_alpha(index_rudder);
      lower_delta_alpha_(index_rudder) =
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
      upper_delta_u_(index_rudder) =
          _max_u * _max_usqrtterm - _RTdata.feedback_u(index_rudder);

      lower_delta_u_(index_rudder) =
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
        upper_delta_alpha_(index_tf) = 0;
        lower_delta_alpha_(index_tf) = 0;
        upper_delta_u_(index_tf) =
            std::min(v_twinfixeddata[i].max_thrust_positive - _u0,
                     _Kp * std::pow(_n0 + _maxdn, 2) - _u0);
        lower_delta_u_(index_tf) = _Kp * std::pow(_n0 - _maxdn, 2) - _u0;
      } else if (_maxdnp2n <= _n0 && _n0 < _maxdn) {
        upper_delta_alpha_(index_tf) = 0;
        lower_delta_alpha_(index_tf) = 0;
        upper_delta_u_(index_tf) = _Kp * std::pow(_n0 + _maxdnp2n, 2) - _u0;
        lower_delta_u_(index_tf) = _Kp * std::pow(_n0 - _maxdnp2n, 2) - _u0;
      } else if (0 < _n0 && _n0 < _maxdnp2n) {
        if (_desired_tau(i) < 0) {
          // change the direction
          upper_delta_alpha_(index_tf) = M_PI;
          lower_delta_alpha_(index_tf) = M_PI;
          lower_delta_u_(index_tf) = _Kn * std::pow(_n0 - _maxdnp2n, 2) - _u0;
          upper_delta_u_(index_tf) = lower_delta_u_(index_tf);
        } else {
          upper_delta_alpha_(index_tf) = 0;
          lower_delta_alpha_(index_tf) = 0;
          lower_delta_u_(index_tf) = _Kp * std::pow(_n0 + _maxdnp2n, 2) - _u0;
          upper_delta_u_(index_tf) = lower_delta_u_(index_tf);
        }

      } else if (-_maxdnp2n < _n0 && _n0 < 0) {
        if (_desired_tau(i) < 0) {
          upper_delta_alpha_(index_tf) = 0;
          lower_delta_alpha_(index_tf) = 0;
          lower_delta_u_(index_tf) = _Kn * std::pow(_n0 - _maxdnp2n, 2) - _u0;
          upper_delta_u_(index_tf) = lower_delta_u_(index_tf);
        } else {
          // change the direction
          upper_delta_alpha_(index_tf) = -M_PI;
          lower_delta_alpha_(index_tf) = -M_PI;
          lower_delta_u_(index_tf) = _Kp * std::pow(_n0 + _maxdnp2n, 2) - _u0;
          upper_delta_u_(index_tf) = lower_delta_u_(index_tf);
        }

      } else if (-_maxdn < _n0 && _n0 <= -_maxdnp2n) {
        upper_delta_alpha_(index_tf) = 0;
        lower_delta_alpha_(index_tf) = 0;
        upper_delta_u_(index_tf) = _Kn * std::pow(_n0 - _maxdnp2n, 2) - _u0;
        lower_delta_u_(index_tf) = _Kn * std::pow(_n0 + _maxdnp2n, 2) - _u0;
      } else {
        upper_delta_alpha_(index_tf) = 0;
        lower_delta_alpha_(index_tf) = 0;
        upper_delta_u_(index_tf) =
            std::min(v_twinfixeddata[i].max_thrust_negative - _u0,
                     _Kn * std::pow(_n0 - _maxdn, 2) - _u0);
        lower_delta_u_(index_tf) = _Kn * std::pow(_n0 + _maxdn, 2) - _u0;
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
    delta_u_ = results_.head(m);
    delta_alpha_ = results_.segment(m, m);
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
    _command_u = _feedback_u + delta_u_;
    _command_alpha = _feedback_alpha + delta_alpha_;
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
      _B_alpha(2, i) = -ly_(i) * t_cos + lx_(i) * t_sin;
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
      d_rho_(i) =
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
      d_Balpha_u_.col(i) = (calculateBalphau(alpha_plus, t_u) -
                            calculateBalphau(alpha_minus, t_u)) /
                           (2 * derivative_dx);
    }
  }  // calculateJocobianBalphaU

  // calculate g_deltau and Q_deltau
  void calculateDeltauQ(const vectormd &t_u) {
    vectormd d_utemp = vectormd::Zero();
    d_utemp = t_u.cwiseSqrt();
    g_deltau_ = 1.5 * d_utemp;
    vectormd Q_temp = vectormd::Zero();
    // Q_temp = 0.75 * d_utemp.cwiseInverse();
    Q_temp = 7.5 * d_utemp.cwiseInverse();
    Q_deltau_ = Q_temp.asDiagonal();
  }  // calculateDeltauQ

  // calculate the BalphaU and b
  void calculateb(const vectornd &_tau, const vectornd &_BalphaU) {
    b_ = _tau - _BalphaU;
  }

  // update parameters in thruster allocation for each time step
  void updateTAparameters(controllerRTdata<m, n> &_RTdata) {
    B_alpha_ = calculateBalpha(_RTdata.feedback_alpha);
    // update BalphaU
    _RTdata.BalphaU = calculateBalphau(B_alpha_, _RTdata.feedback_u);

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

  void initializeOSQPVariables() {
    int two_m_m = 2 * m;
    int eight_m_m = 8 * m;

    // assign value to the objective
    for (int i = 0; i != numvar; ++i) {
      osqp_P_i[i] = i;
      osqp_P_p[i] = i;
      osqp_q[i] = 0;
    }
    osqp_P_p[numvar] = numvar;
    for (int i = 0; i != m; ++i) {
      osqp_P_x[i] = 0;
    }
    for (int i = 0; i != m; ++i) {
      osqp_P_x[i + m] = Omega_(i, i);
    }
    for (int i = 0; i != n; ++i) {
      osqp_P_x[i + 2 * m] = Q_(i, i);
    }

    // assign value to the A in OSQP
    osqp_A_p[0] = 0;
    for (int i = 0; i != two_m_m; ++i) {
      int four_m_i = 4 * i;
      osqp_A_p[i + 1] = four_m_i + 4;
      for (int j = 0; j != 3; ++j) {
        osqp_A_x[four_m_i + j] = 0.0;
        osqp_A_i[four_m_i + j] = j;
      }
      osqp_A_x[four_m_i + 3] = 1.0;
      osqp_A_i[four_m_i + 3] = i + 3;
    }
    for (int i = 0; i != n; ++i) {
      int two_m_i = 2 * i;
      osqp_A_p[1 + 2 * m + i] = eight_m_m + 2 + two_m_i;
      osqp_A_x[two_m_i + eight_m_m] = 1.0;
      osqp_A_x[two_m_i + 1 + eight_m_m] = 1.0;
      osqp_A_i[two_m_i + eight_m_m] = i;
      osqp_A_i[two_m_i + 1 + eight_m_m] = two_m_m + 3 + i;
    }

    // assign value to l and u in OSQP
    for (int i = 0; i != numvar; ++i) {
      osqp_l[i] = 0.0;
      osqp_u[i] = 0.0;
    }
    for (int i = 0; i != n; ++i) {
      int index_n = i + numvar;
      osqp_l[index_n] = -OSQP_INFTY;
      osqp_u[index_n] = OSQP_INFTY;
    }

  }  // initializeOSQPVariables

  void initializeOSQPAPI() {
    osqp_flag = 0;
    osqp_settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    osqp_data = (OSQPData *)c_malloc(sizeof(OSQPData));

    // Populate data
    if (osqp_data) {
      osqp_data->n = numvar;
      osqp_data->m = num_constraints;
      osqp_data->P = csc_matrix(osqp_data->n, osqp_data->n, numvar, osqp_P_x,
                                osqp_P_i, osqp_P_p);
      osqp_data->q = osqp_q;
      osqp_data->A = csc_matrix(osqp_data->m, osqp_data->n, A_nnz, osqp_A_x,
                                osqp_A_i, osqp_A_p);
      osqp_data->l = osqp_l;
      osqp_data->u = osqp_u;
    }

    // Define solver settings as default
    if (osqp_settings) {
      osqp_set_default_settings(osqp_settings);
    }
    osqp_flag = osqp_setup(&osqp_work, osqp_data, osqp_settings);
    // if (osqp_flag != 0) CLOG(ERROR, "osqp") << "setup error.";
    if (osqp_flag != 0) std::cout << "osqp setup error." << std::endl;

  }  // initializeOSQPAPI

  // update parameters in QP for each time step
  void updateOSQPparameters() {
    // update objective
    for (int i = 0; i != m; ++i) {
      osqp_q[i] = g_deltau_(i);
      osqp_q[i + m] = d_rho_(i);
      osqp_P_x[i] = Q_deltau_(i, i);
    }

    // update A
    for (int i = 0; i != m; ++i) {
      for (int j = 0; j != n; ++j) {
        osqp_A_x[4 * i + j] = B_alpha_(j, i);
        osqp_A_x[4 * (i + m) + j] = d_Balpha_u_(j, i);
      }
    }

    // update l and u
    for (int i = 0; i != n; ++i) {
      osqp_l[i] = b_(i);
      osqp_u[i] = b_(i);
    }
    for (int i = 0; i != m; ++i) {
      osqp_l[n + i] = lower_delta_u_(i);
      osqp_u[n + i] = upper_delta_u_(i);
      osqp_l[n + m + i] = lower_delta_alpha_(i);
      osqp_u[n + m + i] = upper_delta_alpha_(i);
    }

    osqp_update_P(osqp_work, osqp_P_x, OSQP_NULL, numvar);
    osqp_update_A(osqp_work, osqp_A_x, OSQP_NULL, A_nnz);
    osqp_update_lin_cost(osqp_work, osqp_q);
    osqp_update_bounds(osqp_work, osqp_l, osqp_u);

  }  // updateOSQPparameters

  // solve QP using OSQP solver
  void onestepOSQP() {
    // reset the delta value
    results_.setZero();

    // Solve Problem
    osqp_solve(osqp_work);
    if (osqp_work->info->status_val > 0) {
      for (int i = 0; i != numvar; ++i) results_(i) = osqp_work->solution->x[i];
    } else {
      // CLOG(ERROR, "osqp") << "solver error.";
      std::cout << "osqp solver error." << std::endl;
    }

  }  // onestepOSQP

  // 一元二次方程
  double computeabcvalue(double a, double b, double c, double x) {
    return a * x * x + b * x + c;
  }

  // convert rad to degree
  int rad2degree(double _rad) { return std::round(_rad * 180 / M_PI); }
  // convert degree to rad
  double degree2rad(int _degree) { return _degree * M_PI / 180.0; }

};  // end class thrustallocation

#endif /* _THRUSTALLOCATION_H_*/