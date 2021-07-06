#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

#include <unitree_legged_sdk/vectornav/vectornav.hpp>

#include <Configuration.h>
#include <PnC/A1PnC/A1Interface.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

#include <Eigen/Dense>

using namespace UNITREE_LEGGED_SDK;


class A1LowDriver {
 public:
  A1LowDriver(): control(LeggedType::A1, LOWLEVEL), udp(){
        control.InitCmdData(cmd);
    }

  ~A1LowDriver();

  void UDPSend();
  void UDPRecv();
  void RobotControl();

  void InitializePnC();

  Control control;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  int motiontime = 0;
  float dt = 0.002;     // 0.001~0.01

  A1Interface* interface_;
  A1SensorData* sensordata_;
  A1Command* command_;

  Vectornav* imu_;
  bool imu_b;

  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
  Eigen::VectorXd final_configuration;
  Eigen::VectorXd jpos_des;
  Eigen::VectorXd starting_configuration;
  Eigen::VectorXd pd_tau;

 protected:
  double clamp_value(double in, double min, double max);
  void compute_jpos_des();
  void extract_sensor_data();
  void set_pd_command();
  void set_command();

  double curr_time;
  double start_time;

};