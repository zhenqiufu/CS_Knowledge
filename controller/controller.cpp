

// 收集目标点, 收集当前状态state-6,计算误差,然后PID出推力pub
// m默认手动,自动以后默认在当前,即error为0,如若有定位要求则输入error

#include <dp_core/controllerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
using state_type = Eigen::Matrix<double, 6, 1>;
using vectornd = Eigen::Matrix<double, 3, 1>;
using matrixnld = Eigen::Matrix<double, 3, 10>;
std::vector<double> param_init_position;
std::vector<double> pre_tau = {0, 0, 0};
std::vector<double> tau_max = {40000, 60000, 40000000};  //推力上限
std::vector<double> tau_step = {50, 50, 80};             //推力不畅
int All_Man_prev, All_DP_prev;
int Relative_Order, Absolute_Order;
int All_DP, All_Man, Auto_Surge, Auto_Sway,
    Auto_Yaw;  // 0 手动 1 自动
double Delta_Surge, Delta_Sway, Delta_Yaw;
double target_NED_N = 0, target_NED_E = 0,
       target_NED_Theta = 0;  // NED坐标系下的目标点
// double tau_step = 10;         // PID增量大小
int L = 10;  //积分长度
int test_loop_rate = 20;

matrixnld positionerror_integralmatrix =
    matrixnld::Zero();  // I for position control
matrixnld velocityerror_integralmatrix =
    matrixnld::Zero();  // I for velocity control
std::vector<double> param_target_position, P_PID_P, P_PID_I, V_PID_P, V_PID_I,
    param_target_velocity = {0, 0, 0};
Eigen::Vector3d windload = Eigen::Vector3d::Zero(),
                P_error = Eigen::Vector3d::Zero(),
                target_NED = Eigen::Vector3d::Zero(),
                Delta_body = Eigen::Vector3d::Zero(),  //基于船体坐标系给的指令
    Delta_NED = Eigen::Vector3d::Zero(),  //换算到大地坐标系下指令
    V_error = Eigen::Vector3d::Zero(),    //
    velocity_allowed_error,               //
    position_allowed_error,               //
    eigen_init_position = Eigen::Vector3d::Zero();
state_type state = state_type::Zero();
Eigen::Matrix<double, 3, 3> B2G_Rotation = Eigen::Matrix<double, 3, 3>::Zero(),
                            G2B_Rotation = Eigen::Matrix<double, 3, 3>::Zero();

void simulator_callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  for (int i = 0; i < 6; ++i) {
    state(i) = msg->data[i];
  }
}
// // linear damping
// void compensatelineardamping(vectornd &_tau, const vectornd &_desired_speed)
// {
//   for (int i = 0; i != n; ++i)
//     _tau(i) += lineardamping(i, i) * _desired_speed(i);
// }
// restrict the desired force to some value

vectornd v_min_output, v_max_output;
double restrictdesiredforce(double _input, double _min, double _max) {
  double t_input = _input;
  if (_input > _max) t_input = _max;
  if (_input < _min) t_input = _min;
  return t_input;
}

void restrictdesiredforce(vectornd &_desiredforce) {
  v_min_output(0) = -30000;
  v_min_output(1) = -30000;
  v_min_output(2) = -300000;
  v_max_output(0) = 30000;
  v_max_output(1) = 30000;
  v_max_output(2) = 300000;
  for (int i = 0; i != 3; ++i)
    _desiredforce(i) = restrictdesiredforce(_desiredforce(i), v_min_output(i),
                                            v_max_output(i));
}  // restrictdesiredforce

// calculate the Integral error with moving window
vectornd updatepositionIntegralMatrix(const vectornd &_error) {
  matrixnld t_integralmatrix = matrixnld::Zero();
  int index = L - 1;
  t_integralmatrix.leftCols(index) =
      positionerror_integralmatrix.rightCols(index);
  // t_integralmatrix.col(index) = sample_time * _error;
  t_integralmatrix.col(index) = _error;
  positionerror_integralmatrix = t_integralmatrix;
  return positionerror_integralmatrix.rowwise().mean();
}  // updatepositionIntegralMatrix

// // calculate the Integral error with moving window
// vectornd updatevelocityIntegralMatrix(const vectornd &_derror) {
//   matrixnld t_integralmatrix = matrixnld::Zero();
//   int index = L - 1;
//   t_integralmatrix.leftCols(index) =
//       velocityerror_integralmatrix.rightCols(index);
//   t_integralmatrix.col(index) = _derror;
//   velocityerror_integralmatrix = t_integralmatrix;
//   return velocityerror_integralmatrix.rowwise().mean();
// }  // updatevelocityIntegralMatrix

// compare the real time position error with the allowed error
bool comparepositionerror(const vectornd &_error) {
  position_allowed_error(0) = 0.1;
  position_allowed_error(1) = 0.1;
  position_allowed_error(2) = M_PI / 1800;

  for (int i = 0; i != 3; ++i)
    if (std::abs(_error(i)) > position_allowed_error(i)) return false;
  return true;
}
// compare the real time velocity error with the allowed error
bool comparevelocityerror(const vectornd &_derror) {
  velocity_allowed_error(0) = 0.1;
  velocity_allowed_error(1) = 0.1;
  velocity_allowed_error(2) = M_PI / 180;
  for (int i = 0; i != 3; ++i)
    if (std::abs(_derror(i)) > velocity_allowed_error(i)) return false;
  return true;
}

void dynamic_callback(dp_core::controllerConfig &config, uint32_t level) {
  All_Man = (int)config.All_Man;
  All_DP = (int)config.All_DP;
  Auto_Surge = (int)config.Auto_Surge;
  Auto_Sway = (int)config.Auto_Sway;
  Auto_Yaw = (int)config.Auto_Yaw;

  if (All_Man == 1 && All_Man_prev == 0) {
    All_DP = 0;
    config.All_DP = false;
    Auto_Surge = 0;
    config.Auto_Surge = false;
    Auto_Sway = 0;
    config.Auto_Sway = false;
    Auto_Yaw = 0;
    config.Auto_Yaw = false;
  }
  if (All_DP == 1 && All_DP_prev == 0) {
    All_Man = 0;
    config.All_Man = false;
    Auto_Surge = 1;
    config.Auto_Surge = true;
    Auto_Sway = 1;
    config.Auto_Sway = true;
    Auto_Yaw = 1;
    config.Auto_Yaw = true;
  }
  if (Auto_Surge == 1 || Auto_Sway == 1 || Auto_Yaw == 1) {
    All_Man = 0;
    config.All_Man = false;
    All_DP = 1;
    config.All_DP = true;
  }
  if (Auto_Surge == 0 && Auto_Sway == 0 && Auto_Yaw == 0) {
    All_DP = 0;
    config.All_DP = false;
  }

  All_Man_prev = All_Man;
  All_DP_prev = All_DP;

  std::cout << "All_Man:" << All_Man << "  "
            << "All_DP:" << All_DP << "  "
            << "3dof: " << Auto_Surge << Auto_Sway << Auto_Yaw << std::endl;

  // Relative_Order = (int)config.Relative_Order;
  Delta_body(0) = config.Delta_Surge;
  Delta_body(1) = config.Delta_Sway;
  Delta_body(2) = (config.Delta_Yaw) * M_PI / 180;

  std::cout << "Delta_Surge:" << Delta_Surge << "  "
            << "Delta_Sway:" << Delta_Sway << "  "
            << "Delta_Yaw: " << Delta_Yaw << std::endl;

  if (config.record_here) {
    config.NED_N = state(0);
    config.NED_E = state(1);
    config.NED_Theta = (state(2) / M_PI * 180);
    eigen_init_position(0) = state(0);
    eigen_init_position(1) = state(1);
    eigen_init_position(2) = state(2);
    config.Delta_Surge = 0;
    config.Delta_Sway = 0;
    config.Delta_Yaw = 0;
    config.record_here = false;
    Delta_body(0) = 0;
    Delta_body(1) = 0;
    Delta_body(2) = 0;
  }
}

int main(int argv, char **argc) {
  ros::init(argv, argc, "controller");
  ros::NodeHandle n;
  ros::NodeHandle private_nh;
  ros::Subscriber sub_vessel_data =
      n.subscribe("nonlinearObserver", 1, simulator_callback);
  n.getParam("target_point", param_target_position);
  n.getParam("target_velocity", param_target_velocity);
  n.getParam("P_PID_P", P_PID_P);
  n.getParam("P_PID_I", P_PID_I);
  n.getParam("V_PID_P", V_PID_P);
  n.getParam("V_PID_I", V_PID_I);
  n.getParam("init_point", param_init_position);
  n.getParam("test_loop_rate", test_loop_rate);
  // n.getParam("V_PID", v_PID);
  eigen_init_position(0) = param_init_position[0];
  eigen_init_position(1) = param_init_position[1];
  eigen_init_position(2) = param_init_position[2];

  ros::Publisher pub_thruster =
      n.advertise<std_msgs::Float64MultiArray>("controller", 1000);
  std_msgs::Float64MultiArray thruster_msg;

  for (int i = 0; i < 3; ++i) {
    thruster_msg.data.push_back(0);
  }

  dynamic_reconfigure::Server<dp_core::controllerConfig> server;
  server.setCallback(boost::bind(&dynamic_callback, _1, _2));

  ros::Rate loop_rate(test_loop_rate);
  while (ros::ok()) {
    //数值计算
    Eigen::Vector3d _tau = Eigen::Vector3d::Zero();
    // cal error
    if (All_DP) {  // DP模式
      B2G_Rotation.setIdentity();
      G2B_Rotation.setIdentity();
      double cvalue = std::cos(state(2));
      double svalue = std::sin(state(2));

      G2B_Rotation(0, 0) = cvalue;
      G2B_Rotation(1, 1) = cvalue;
      G2B_Rotation(0, 1) = svalue;
      G2B_Rotation(1, 0) = -svalue;

      B2G_Rotation(0, 0) = cvalue;
      B2G_Rotation(1, 1) = cvalue;
      B2G_Rotation(0, 1) = -svalue;
      B2G_Rotation(1, 0) = svalue;

      Delta_NED = B2G_Rotation * Delta_body;  //将delta转到NED
      // std::cout << "Delta_body" << Delta_body << std::endl;
      target_NED = Delta_NED + eigen_init_position;  //根据大地NED设置目标点
      // std::cout << "target_NED" << target_NED << std::endl;
      // target_NED = init_position
      // +Delta_NED;//可以加一个记录当前点的按钮,然后基于当前点求新的大地目标

      P_error = G2B_Rotation * (target_NED - state.head(3));  //奔向目标点
      // std::cout << "P_error" << P_error << std::endl;
    }

    if (All_Man) {  //手动模式,默认无误差,PID的力为0
      P_error = Eigen::Vector3d::Zero();
    }
    for (int i = 0; i < 3; ++i) {
      V_error(i) = param_target_velocity[i] - state(i + 3);
    }
    // std::cout << "Delta_body" << Delta_body << std::endl;
    // std::cout << "target_NED: " << target_NED << std::endl;

    std::cout << "P_error: " << P_error(0) << "   " << P_error(1) << "  "
              << P_error(2) * 180 / M_PI << std::endl;

    if (P_error(0) > 1) P_error(0) = 1;
    if (P_error(1) > 1) P_error(1) = 1;
    if (P_error(2) > M_PI / 60) P_error(2) = M_PI / 60;

    if (P_error(0) < -1) P_error(0) = -1;
    if (P_error(1) < -1) P_error(1) = -1;
    if (P_error(2) < -M_PI / 60) P_error(2) = -M_PI / 60;

    // position control
    vectornd position_error_integral = updatepositionIntegralMatrix(P_error);

    if (!comparepositionerror(P_error)) {
      for (int i = 0; i < 3; ++i) {
        _tau(i) += P_PID_P[i] * P_error(i)  // proportional term
                   + P_PID_I[i] * position_error_integral(i);  // integral term
      }
    }

    if (!comparevelocityerror(V_error)) {
      for (int i = 0; i < 3; ++i) {
        _tau(i) += V_PID_P[i] * V_error(i);  // proportional term
        //  + V_PID_I[i] *
        position_error_integral(i);
        //  // integral term
      }
    }

    // 增量式
    for (int i = 0; i < 3; ++i) {
      if (_tau(i) * pre_tau[i] < 0) {
        _tau(i) = 0;
      }
      if (_tau(i) - pre_tau[i] > tau_step[i]) {
        _tau(i) = pre_tau[i] + tau_step[i];
      }
      if (pre_tau[i] - _tau(i) > tau_step[i]) {
        _tau(i) = pre_tau[i] - tau_step[i];
      }
    }
    //约束
    for (int i = 0; i < 3; ++i) {
      if (_tau(i) > tau_max[i]) {
        _tau(i) = tau_max[i];
      }
      if (_tau(i) < -tau_max[i]) {
        _tau(i) = -tau_max[i];
      }
    }

    //赋值
    thruster_msg.data[0] = 0;
    thruster_msg.data[1] = 0;
    thruster_msg.data[2] = 0;
    // restrictdesiredforce(_tau);
    if (Auto_Surge) thruster_msg.data[0] = _tau(0);
    if (Auto_Sway) thruster_msg.data[1] = _tau(1);
    if (Auto_Yaw) thruster_msg.data[2] = _tau(2);

    pre_tau[0] = _tau(0);
    pre_tau[1] = _tau(1);
    pre_tau[2] = _tau(2);

    // std::cout << "waveloadRTdata" << waveload_msg << std::endl;
    // std::cout << "********************" << std::endl;
    //发布
    pub_thruster.publish(thruster_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  //}

  // velocity control
  // velocityloop(d_tau, _derror);
  // vectornd velocity_error_integral = updatevelocityIntegralMatrix(_derror);
  // // if (!comparevelocityerror(_derror)) {
  // for (int i = 0; i != n; ++i)
  //   _tau(i) +=
  //       velocity_pids(0, i) * V_error(i)  // proportional term
  //       + velocity_pids(1, i) * velocity_error_integral(i);  // integral
  //       term
  //}

  // Damping compensation
  // compensatelineardamping(d_tau, _desired_speed);

  // restrict desired force
  // restrictdesiredforce(d_tau);
}