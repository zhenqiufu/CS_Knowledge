/*
 * @Author: Zhenqiu Fu
 * @Date: 2020-06-04 16:28:33
 * @Last Modified by:   Zhenqiu Fu
 * @Last Modified time: 2020-06-04 16:28:33
 */

// TAIANKOU 推力分配节点 订阅目标推力 pub分配后的推力

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "gnuplot-iostream.h"
#include "thrustallocation.h"
// #include "thrustallocation_osqp.h"

double taux, tauy, tauz;
int test_loop_rate = 20;

// test thrust allocation for 4 propellers (fully actuated)
// void testonestepthrustallocation() {
// std::cout << "command_alpha: \n"
//           << _controllerRTdata.command_alpha << std::endl;
// std::cout << "upper_delta_alpha: \n"
//           << _thrustallocation.upper_delta_alpha() << std::endl;
// std::cout << "lower_delta_alpha: \n"
//           << _thrustallocation.lower_delta_alpha() << std::endl;
// std::cout << "upper_delta_u: \n"
//           << _thrustallocation.upper_delta_u() << std::endl;
// std::cout << "lower_delta_u: \n"
//           << _thrustallocation.lower_delta_u() << std::endl;
// std::cout << "Q: \n" << _thrustallocation.Q() << std::endl;
// std::cout << "Omega: \n" << _thrustallocation.Omega() << std::endl;
// std::cout << "Q_deltau: \n" << _thrustallocation.Q_deltau() << std::endl;
// std::cout << "g_deltau: \n" << _thrustallocation.g_deltau() << std::endl;
// std::cout << "d_rho: \n" << _thrustallocation.d_rho() << std::endl;
// std::cout << "B_alpha: \n" << _thrustallocation.B_alpha() << std::endl;
// std::cout << "d_Balpha_u: \n" << _thrustallocation.d_Balpha_u() << std::endl;
// std::cout << "lx: \n" << _thrustallocation.lx() << std::endl;
// }  // testonestepthrustallocation

// test thrust allocation for 3 propellers (fully actuated)

void controller_callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  //
  taux = msg->data[0];
  tauy = msg->data[1];
  tauz = msg->data[2];
}

int main(int argv, char **argc) {
  ros::init(argv, argc, "thruster");
  // ros::NodeHandle n;
  ros::NodeHandle private_nh;
  private_nh.getParam("test_loop_rate", test_loop_rate);

  // ros::Subscriber sub_windload = n.subscribe("windload", 1,
  // wind_load_callback);
  ros::Subscriber sub_thrustload =
      private_nh.subscribe("controller", 1, controller_callback);

  // n.getParam("wind_switch", wind_switch);

  ros::Publisher pub_BalphaU =
      private_nh.advertise<std_msgs::Float64MultiArray>("thruster", 1000);
  std_msgs::Float64MultiArray thruster_msg;

  for (int i = 0; i < 3; ++i) {
    thruster_msg.data.push_back(0);  //前三个balphau,然后转度4,角度4
  }
  /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
  const int m = 3;
  const int n = 3;
  constexpr ACTUATION index_actuation = ACTUATION::FULLYACTUATED;

  std::vector<int> index_thrusters{1, 2, 2};

  int num_tunnel =
      std::count(index_thrusters.begin(), index_thrusters.end(), 1);

  int num_azimuth =
      std::count(index_thrusters.begin(), index_thrusters.end(), 2);

  int num_mainrudder =
      std::count(index_thrusters.begin(), index_thrusters.end(), 3);

  int num_twinfixed =
      std::count(index_thrusters.begin(), index_thrusters.end(), 4);

  thrustallocationdata _thrustallocationdata{
      5000,            // Q_surge
      5000,            // Q_sway
      10000,           // Q_yaw
      num_tunnel,      // num_tunnel
      num_azimuth,     // num_azimuth
      num_mainrudder,  // num_mainrudder
      num_twinfixed,   // num_twinfixed
      index_thrusters  // index_thrusters
  };

  std::vector<tunnelthrusterdata> v_tunnelthrusterdata;
  v_tunnelthrusterdata.reserve(num_tunnel);
  v_tunnelthrusterdata.push_back({135, 0, 20e-2, 5e-2, 50, 1000, 333, 1.53});

  std::cout << "node 1: \n" << std::endl;
  //////////////////////////////////////////////////////////////实船数据
  // #1侧推
  // v_tunnelthrusterdata.push_back({
  //     75.8,    // lx
  //     0,       // ly
  //     6.5e-2,  // K_positive
  //     6.5e-2,  // K_negative
  //     50,      // max_delta_rotation
  //     1000,    // max_rotation
  //     300000,  // max_thrust_positive
  //     -200000  // max_thrust_negative
  // });

  // v_tunnelthrusterdata.push_back({
  //     135,     // lx
  //     0,       // ly
  //     6.5e-2,  // K_positive
  //     6.5e-2,  // K_negative
  //     50,      // max_delta_rotation
  //     1000,    // max_rotation
  //     300000,  // max_thrust_positive
  //     -200000  // max_thrust_negative
  // });
  // // #2侧推
  // // v_tunnelthrusterdata.push_back({
  // //     73.8,    // lx
  // //     0,       // ly
  // //     6.5e-2,  // K_positive
  // //     6.5e-2,  // K_negative
  // //     50,      // max_delta_rotation
  // //     1000,    // max_rotation
  // //     300000,  // max_thrust_positive
  // //     -200000  // max_thrust_negative
  // // });

  // std::vector<azimuththrusterdata> v_azimuththrusterdata;
  // v_azimuththrusterdata.reserve(num_azimuth);
  // v_azimuththrusterdata.push_back({
  //     -2.5,          // lx
  //     8,             // ly
  //     12.5,          // K
  //     5,             // max_delta_rotation
  //     155,           // max rotation
  //     0,             // min_rotation
  //     0.1277,        // max_delta_alpha
  //     7 * M_PI / 6,  // max_alpha
  //     -M_PI / 6,     // min_alpha
  //     1000000,       // max_thrust
  //     2e-3           // min_thrust
  // });
  // v_azimuththrusterdata.push_back({
  //     -2.5,           // lx
  //     -8,             // ly
  //     12.5,           // K
  //     5,              // max_delta_rotation
  //     155,            // max rotation
  //     0,              // min_rotation
  //     0.1277,         // max_delta_alpha
  //     M_PI / 6,       // max_alpha
  //     -7 * M_PI / 6,  // min_alpha
  //     1000000,        // max_thrust
  //     2e-3            // min_thrust
  // });

  //////////////////////////////////////////////////////////
  std::vector<ruddermaindata> v_ruddermaindata;
  std::vector<twinfixedthrusterdata> v_twinfixeddata;

  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  v_azimuththrusterdata.reserve(num_azimuth);
  v_azimuththrusterdata.push_back({
      -1.893,         // lx
      -8,             // ly
      12.5,           // K
      10,             // max_delta_rotation
      155,            // max rotation
      1,              // min_rotation
      0.1277 * 10,    // max_delta_alpha
      M_PI / 6,       // max_alpha
      -7 * M_PI / 6,  // min_alpha
      20,             // max_thrust
      0.002           // min_thrust
  });
  v_azimuththrusterdata.push_back({
      -1.893,        // lx
      8,             // ly
      12.5,          // K
      10,            // max_delta_rotation
      155,           // max rotation
      1,             // min_rotation
      0.1277 * 10,   // max_delta_alpha
      7 * M_PI / 6,  // max_alpha
      -M_PI / 6,     // min_alpha
      20,            // max_thrust
      0.002          // min_thrust
  });

  // std::vector<ruddermaindata> v_ruddermaindata;
  // std::vector<twinfixedthrusterdata> v_twinfixeddata;

  controllerRTdata<m, n> _controllerRTdata{
      STATETOGGLE::IDLE,                    // state_toggle
      Eigen::Matrix<double, n, 1>::Zero(),  // tau
      Eigen::Matrix<double, n, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, m, 1>::Zero(),  // command_u
      Eigen::Matrix<int, m, 1>::Zero(),     // command_rotation
      Eigen::Matrix<double, m, 1>::Zero(),  // command_alpha
      Eigen::Matrix<int, m, 1>::Zero(),     // command_alpha_deg
      Eigen::Matrix<double, m, 1>::Zero(),  // feedback_u
      Eigen::Matrix<int, m, 1>::Zero(),     // feedback_rotation
      Eigen::Matrix<double, m, 1>::Zero(),  // feedback_alpha
      Eigen::Matrix<int, m, 1>::Zero()      // feedback_alpha_deg
  };

  // controllerRTdata<m, n> _controllerRTdata{
  //     STATETOGGLE::IDLE,                                      // state_toggle
  //     (Eigen::Matrix<double, n, 1>() << 0, 0, 1).finished(),  // tau
  //     Eigen::Matrix<double, n, 1>::Zero(),                    // BalphaU
  //     (Eigen::Matrix<double, m, 1>() <<0.5, 0, 1).finished(),  // command_u
  //     // vectormi()::Zero(),
  //     (Eigen::Matrix<int, m, 1>() << 100, 500, 400, 300)
  //         .finished(),  // command_rotation
  //     (Eigen::Matrix<double, m, 1>() << M_PI / 2, -M_PI / 2, M_PI * 2 / 3, 0)
  //         .finished(),                   // command_alpha
  //     Eigen::Matrix<int, m, 1>::Zero(),  // command_alpha_deg
  //     (Eigen::Matrix<double, m, 1>() << 0, 0.5, 0, 1).finished(),  //
  //     feedback_u
  //     // vectormi()::Zero(),                    // feedback_rotation
  //     (Eigen::Matrix<int, m, 1>() << 100, 500, 400, 300).finished(),
  //     (Eigen::Matrix<double, m, 1>() << M_PI / 2, -M_PI / 2, M_PI * 2 / 3, 0)
  //         .finished(),                  // feedback_alpha
  //     Eigen::Matrix<int, m, 1>::Zero()  // feedback_alpha_deg

  // };
  std::cout << "node 2: \n" << std::endl;

  thrustallocation<m, index_actuation, n> _thrustallocation(
      _thrustallocationdata, v_tunnelthrusterdata, v_azimuththrusterdata,
      v_ruddermaindata, v_twinfixeddata);
  std::cout << "node 3: \n" << std::endl;
  _thrustallocation.initializapropeller(_controllerRTdata);
  std::cout << "node 4: \n" << std::endl;
  _thrustallocation.setQ(CONTROLMODE::DYNAMICPOSITION);
  std::cout << "node 5: \n" << std::endl;

  _thrustallocation.initializapropeller(_controllerRTdata);
  /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
  ros::Rate loop_rate(test_loop_rate);
  std::cout << "node 6: \n" << std::endl;

  while (ros::ok()) {
    _controllerRTdata.tau(0) = taux;
    _controllerRTdata.tau(1) = tauy;
    _controllerRTdata.tau(2) = tauz;

    _controllerRTdata.feedback_rotation = _controllerRTdata.command_rotation;
    _controllerRTdata.feedback_alpha_deg = _controllerRTdata.command_alpha_deg;
    _thrustallocation.onestepthrustallocation(_controllerRTdata);
    // std::cout << "command_alpha: \n"
    //           << _controllerRTdata.command_alpha << std::endl;
    // std::cout << "upper_delta_alpha: \n"
    //           << _thrustallocation.upper_delta_alpha() << std::endl;
    // std::cout << "lower_delta_alpha: \n"
    //           << _thrustallocation.lower_delta_alpha() << std::endl;
    // std::cout << "upper_delta_u: \n"
    //           << _thrustallocation.upper_delta_u() << std::endl;
    // std::cout << "lower_delta_u: \n"
    //           << _thrustallocation.lower_delta_u() << std::endl;
    // std::cout << "Q: \n" << _thrustallocation.Q() << std::endl;
    // std::cout << "Omega: \n" << _thrustallocation.Omega() << std::endl;
    // std::cout << "Q_deltau: \n" << _thrustallocation.Q_deltau() << std::endl;
    // std::cout << "g_deltau: \n" << _thrustallocation.g_deltau() << std::endl;
    // std::cout << "d_rho: \n" << _thrustallocation.d_rho() << std::endl;
    // std::cout << "B_alpha: \n" << _thrustallocation.B_alpha() << std::endl;
    // std::cout << "d_Balpha_u: \n"
    //           << _thrustallocation.d_Balpha_u() << std::endl;
    // std::cout << "lx: \n" << _thrustallocation.lx() << std::endl;
    // testonestepthrustallocation();

    //赋值

    for (int j = 0; j < 3; ++j) {
      thruster_msg.data[j] = _controllerRTdata.BalphaU(j);
    }
    // for (int ij = 0; ij < 3; ++ij) {
    //   thruster_msg.data[ij + 3] = _controllerRTdata.command_rotation(ij);
    //   thruster_msg.data[ij + 6] = _controllerRTdata.command_alpha_deg(ij);
    // }

    //发布
    pub_BalphaU.publish(thruster_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}