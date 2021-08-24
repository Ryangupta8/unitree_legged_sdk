#include <unitree_legged_sdk/A1PositionDriver.hpp>

using namespace UNITREE_LEGGED_SDK;

A1PositionDriver::~A1PositionDriver() {
  delete imu_;
}

double A1PositionDriver::clamp_value(double in,
                                     double min,
                                     double max) {
  if (in >= max) return max;
  else if (in <= min) return min;
  else return in;
}

void A1PositionDriver::Initialize() {
  imu_ = new Vectornav();
  imu_b = false;

  // Values used to get the robot into proper starting configuation
  // upon startup of experiment
  starting_q = Eigen::VectorXd::Zero(12);
  starting_q << 0, 0.7854, -1.57, 0, 0.7854, -1.57, 0, 0.7854, -1.57 0, 0.7854, -1.57;
  // finish_q = Eigen::VectorXd::Zero(12);
  // des_q = Eigen::VectorXd::Zero(12);

  // Helpers for smooth_changing
  curr_time = 0.;
  start_time = 0.;

  kp_ = Eigen::VectorXd::Zero(12);
  kd_ = Eigen::VectorXd::Zero(12);

  // Sensor Data Holders
  q_ = Eigen::VectorXd::Zero(12);
  qdot_ = Eigen::VectorXd::Zero(12);
  tau_ = Eigen::VectorXd::Zero(12);
  foot_force_ = Eigen::VectorXi::Zero(4);

  rpy_ = Eigen::VectorXd::Zero(3);
  acc_ = Eigen::VectorXd::Zero(3);
  ang_vel_ = Eigen::VectorXd::Zero(3);
  prev_rpy_ = Eigen::VectorXd::Zero(3);
  prev_acc_ = Eigen::VectorXd::Zero(3);
  prev_ang_vel_ = Eigen::VectorXd::Zero(3);

}


void A1PositionDriver::UDPRecv() {
  udp.Recv();
}

void A1PositionDriver::UDPSend() {
  udp.Send();
}

void A1PositionDriver::extract_sensor_data() {
  q_[0] = state.motorState[FL_0].q;
  q_[1] = state.motorState[FL_1].q;
  q_[2] = state.motorState[FL_2].q;
  q_[3] = state.motorState[FR_0].q;
  q_[4] = state.motorState[FR_1].q;
  q_[5] = state.motorState[FR_2].q;
  q_[6] = state.motorState[RL_0].q;
  q_[7] = state.motorState[RL_1].q;
  q_[8] = state.motorState[RL_2].q;
  q_[9] = state.motorState[RR_0].q;
  q_[10] = state.motorState[RR_1].q;
  q_[11] = state.motorState[RR_2].q;

  qdot_[0] = state.motorState[FL_0].dq;
  qdot_[1] = state.motorState[FL_1].dq;
  qdot_[2] = state.motorState[FL_2].dq;
  qdot_[3] = state.motorState[FR_0].dq;
  qdot_[4] = state.motorState[FR_1].dq;
  qdot_[5] = state.motorState[FR_2].dq;
  qdot_[6] = state.motorState[RL_0].dq;
  qdot_[7] = state.motorState[RL_1].dq;
  qdot_[8] = state.motorState[RL_2].dq;
  qdot_[9] = state.motorState[RR_0].dq;
  qdot_[10] = state.motorState[RR_1].dq;
  qdot_[11] = state.motorState[RR_2].dq;

  tau_[0] = state.motorState[FL_0].tauEst;
  tau_[1] = state.motorState[FL_1].tauEst;
  tau_[2] = state.motorState[FL_2].tauEst;
  tau_[3] = state.motorState[FR_0].tauEst;
  tau_[4] = state.motorState[FR_1].tauEst;
  tau_[5] = state.motorState[FR_2].tauEst;
  tau_[6] = state.motorState[RL_0].tauEst;
  tau_[7] = state.motorState[RL_1].tauEst;
  tau_[8] = state.motorState[RL_2].tauEst;
  tau_[9] = state.motorState[RR_0].tauEst;
  tau_[10] = state.motorState[RR_1].tauEst;
  tau_[11] = state.motorState[RR_2].tauEst;

  // imu running at a slower rate than controller
  // So, grab the new value when it is available
  imu_b = imu_->getData();
  if (imu_b) {
    rpy_ = imu_->raRPY;
    acc_ = imu_->raAcc;
    ang_vel_ = imu_->raGyro;
  } else {
    rpy_ = prev_rpy_;
    acc_ = prev_acc_;
    ang_vel_ = prev_ang_vel_;
  }

  for (int i=0; i<4; ++i) {
    foot_force_[i] = state.footForce[i];
  }
}


A1PositionDriver::set_initial_command() {
  // Set the robot into its desired starting position
  cmd.motorCmd[FL_0].q = starting_q[0];
  cmd.motorCmd[FL_1].q = starting_q[1];
  cmd.motorCmd[FL_2].q = starting_q[2];
  cmd.motorCmd[FR_0].q = starting_q[3];
  cmd.motorCmd[FR_1].q = starting_q[4];
  cmd.motorCmd[FR_2].q = starting_q[5];
  cmd.motorCmd[RL_0].q = starting_q[6];
  cmd.motorCmd[RL_1].q = starting_q[7];
  cmd.motorCmd[RL_2].q = starting_q[8];
  cmd.motorCmd[RR_0].q = starting_q[9];
  cmd.motorCmd[RR_1].q = starting_q[10];
  cmd.motorCmd[RR_2].q = starting_q[11];

  cmd.motorCmd[FL_0].dq = 0;
  cmd.motorCmd[FL_1].dq = 0;
  cmd.motorCmd[FL_2].dq = 0;
  cmd.motorCmd[FR_0].dq = 0;
  cmd.motorCmd[FR_1].dq = 0;
  cmd.motorCmd[FR_2].dq = 0;
  cmd.motorCmd[RL_0].dq = 0;
  cmd.motorCmd[RL_1].dq = 0;
  cmd.motorCmd[RL_2].dq = 0;
  cmd.motorCmd[RR_0].dq = 0;
  cmd.motorCmd[RR_1].dq = 0;
  cmd.motorCmd[RR_2].dq = 0;

  cmd.motorCmd[FL_0].Kp = kp_[0];
  cmd.motorCmd[FL_1].Kp = kp_[1];
  cmd.motorCmd[FL_2].Kp = kp_[2];
  cmd.motorCmd[FR_0].Kp = kp_[3];
  cmd.motorCmd[FR_1].Kp = kp_[4];
  cmd.motorCmd[FR_2].Kp = kp_[5];
  cmd.motorCmd[RL_0].Kp = kp_[6];
  cmd.motorCmd[RL_1].Kp = kp_[7];
  cmd.motorCmd[RL_2].Kp = kp_[8];
  cmd.motorCmd[RR_0].Kp = kp_[9];
  cmd.motorCmd[RR_1].Kp = kp_[10];
  cmd.motorCmd[RR_2].Kp = kp_[11];

  cmd.motorCmd[FL_0].Kd = kd_[0];
  cmd.motorCmd[FL_1].Kd = kd_[1];
  cmd.motorCmd[FL_2].Kd = kd_[2];
  cmd.motorCmd[FR_0].Kd = kd_[3];
  cmd.motorCmd[FR_1].Kd = kd_[4];
  cmd.motorCmd[FR_2].Kd = kd_[5];
  cmd.motorCmd[RL_0].Kd = kd_[6];
  cmd.motorCmd[RL_1].Kd = kd_[7];
  cmd.motorCmd[RL_2].Kd = kd_[8];
  cmd.motorCmd[RR_0].Kd = kd_[9];
  cmd.motorCmd[RR_1].Kd = kd_[10];
  cmd.motorCmd[RR_2].Kd = kd_[11];

}


A1PositionDriver::RobotControl() {
  if (curr_time <= 0.002) Initialize();

  curr_time += 0.002;
  motiontime ++;
  // Get Robot State
  udp.GetRecv(state);
  // Get Sensor Data from Unitree Structure
  extract_sensor_data();

  // Perform the Control here
  // if (curr_time >= 5) {
  //   end2end.getPositionCommands();
  //   set_command();
  // } else {
    set_initial_command();
  // }

  cmd.motorCmd[FR_0].tau = -0.65f;
  cmd.motorCmd[FL_0].tau = +0.65f;
  cmd.motorCmd[RR_0].tau = -0.65f;
  cmd.motorCmd[RL_0].tau = +0.65f;

  control.PositionLimit(cmd);
  control.PowerProtect(cmd, state, 1);
  control.PositionProtect(cmd, state, 0.087);

  udp.SetSend(cmd);
}


