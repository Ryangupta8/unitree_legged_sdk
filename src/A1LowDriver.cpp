#include <unitree_legged_sdk/A1LowDriver.hpp>

using namespace UNITREE_LEGGED_SDK;


A1LowDriver::~A1LowDriver() {
  delete interface_;
  delete sensordata_;
  delete command_;
  delete imu_;
}

double A1LowDriver::clamp_value(double in,
                                double min,
                                double max) {
    if (in >= max) return max;
    else if (in <= min) return min;
    else return in;
}


void A1LowDriver::InitializePnC() {
  // PnC Structures
  interface_ = new A1Interface();
  sensordata_ = new A1SensorData();
  command_ = new A1Command();
  // IMU Vars
  imu_ = new Vectornav();
  imu_b = false;
  // Low level PD Gains
  kp_ = Eigen::VectorXd::Zero(12);
  kd_ = Eigen::VectorXd::Zero(12);
  // Desired standing config
  final_configuration = Eigen::VectorXd::Zero(12);
  // For using smooth_changing
  jpos_des = Eigen::VectorXd::Zero(12);
  // Robot startup config
  starting_configuration = Eigen::VectorXd::Zero(12);
  // Hold low level PD Value
  pd_tau = Eigen::VectorXd::Zero(12);
  // Helpers for smooth_changing
  curr_time = 0.;
  start_time = 0.;

  try {
  YAML::Node simulation_cfg =
      YAML::LoadFile("/home/knapsack/project/locomotion/src/PnC/Config/A1/TEST/REAL.yaml");
  myUtils::readParameter(simulation_cfg, "initial_configuration", final_configuration);
  YAML::Node control_cfg = simulation_cfg["control_configuration"];
  myUtils::readParameter(control_cfg, "kp", kp_);
  myUtils::readParameter(control_cfg, "kd", kd_);
} catch (std::runtime_error& e) {
  std::cout << "Error reading parameter [" << e.what() << "] at file: ["
            << __FILE__ << "]" << std::endl
            << std::endl;
}

// myUtils::pretty_print(final_configuration, std::cout, "initial config");

}

void A1LowDriver::UDPRecv()
{ 
  udp.Recv();
}

void A1LowDriver::UDPSend()
{  
  udp.Send();
}

void A1LowDriver::extract_sensor_data() {
// Gather and set  A1SensorData
      // current robot q
  sensordata_->q[0] = state.motorState[FL_0].q;
  sensordata_->q[1] = state.motorState[FL_1].q;
  sensordata_->q[2] = state.motorState[FL_2].q;
  sensordata_->q[3] = state.motorState[FR_0].q;
  sensordata_->q[4] = state.motorState[FR_1].q;
  sensordata_->q[5] = state.motorState[FR_2].q;
  sensordata_->q[6] = state.motorState[RL_0].q;
  sensordata_->q[7] = state.motorState[RL_1].q;
  sensordata_->q[8] = state.motorState[RL_2].q;
  sensordata_->q[9] = state.motorState[RR_0].q;
  sensordata_->q[10] = state.motorState[RR_1].q;
  sensordata_->q[11] = state.motorState[RR_2].q;

  if (sensordata_->q.norm() >= 1.0) starting_configuration = sensordata_->q;
  // myUtils::pretty_print(sensordata_->q, std::cout, "Current q");
  // myUtils::pretty_print(starting_configuration, std::cout, "starting_q");
  // myUtils::pretty_print(final_configuration, std::cout, "final_q");
  // current robot qdot
  sensordata_->qdot[0] = state.motorState[FL_0].dq;
  sensordata_->qdot[1] = state.motorState[FL_1].dq;
  sensordata_->qdot[2] = state.motorState[FL_2].dq;
  sensordata_->qdot[3] = state.motorState[FR_0].dq;
  sensordata_->qdot[4] = state.motorState[FR_1].dq;
  sensordata_->qdot[5] = state.motorState[FR_2].dq;
  sensordata_->qdot[6] = state.motorState[RL_0].dq;
  sensordata_->qdot[7] = state.motorState[RL_1].dq;
  sensordata_->qdot[8] = state.motorState[RL_2].dq;
  sensordata_->qdot[9] = state.motorState[RR_0].dq;
  sensordata_->qdot[10] = state.motorState[RR_1].dq;
  sensordata_->qdot[11] = state.motorState[RR_2].dq;
      // current robot torque
  sensordata_->jtrq[0] = state.motorState[FL_0].tauEst;
  sensordata_->jtrq[1] = state.motorState[FL_1].tauEst;
  sensordata_->jtrq[2] = state.motorState[FL_2].tauEst;
  sensordata_->jtrq[3] = state.motorState[FR_0].tauEst;
  sensordata_->jtrq[4] = state.motorState[FR_1].tauEst;
  sensordata_->jtrq[5] = state.motorState[FR_2].tauEst;
  sensordata_->jtrq[6] = state.motorState[RL_0].tauEst;
  sensordata_->jtrq[7] = state.motorState[RL_1].tauEst;
  sensordata_->jtrq[8] = state.motorState[RL_2].tauEst;
  sensordata_->jtrq[9] = state.motorState[RR_0].tauEst;
  sensordata_->jtrq[10] = state.motorState[RR_1].tauEst;
  sensordata_->jtrq[11] = state.motorState[RR_2].tauEst;

  // IMU Data (returns true when data received)
  while(!imu_b) imu_b = imu_->getData();
  // TODO: Handle the magnetic north offset
  /*sensordata_->imu_rpy = imu_->raRPY;
  sensordata_->imu_acc = imu_->raAcc;
  sensordata_->imu_ang_vel = imu_->raGyro;*/

  // Gather foot force data
  for (int i=0; i<4; ++i) {
    /*sensordata_->foot_force[i] = state.footForce[i];*/
    // sensordata_->foot_force[i] = state.footForceEst[i];
  }
}


void A1LowDriver::set_pd_command() {
  // Do not start controller until we get first sensor data
  if (sensordata_->q.norm() <= 1.) {
    cmd.motorCmd[FL_0].tau = 0.;
    cmd.motorCmd[FL_1].tau = 0.;
    cmd.motorCmd[FL_2].tau = 0.;
    cmd.motorCmd[FR_0].tau = 0.;
    cmd.motorCmd[FR_1].tau = 0.;
    cmd.motorCmd[FR_2].tau = 0.;
    cmd.motorCmd[RL_0].tau = 0.;
    cmd.motorCmd[RL_1].tau = 0.;
    cmd.motorCmd[RL_2].tau = 0.;
    cmd.motorCmd[RR_0].tau = 0.;
    cmd.motorCmd[RR_1].tau = 0.;
    cmd.motorCmd[RR_2].tau = 0.;
  } else {
    // std::cout << "Correct sensor data begins" << std::endl;
    // myUtils::pretty_print(starting_configuration, std::cout, "starting q");
    // myUtils::pretty_print(final_configuration, std::cout, "goal q");
    for(int i=0; i<12; ++i) {
      start_time = curr_time;
      jpos_des[i] =
          myUtils::smooth_changing(starting_configuration[i], final_configuration[i],
                                   5.0, curr_time);
    }
    // myUtils::pretty_print(jpos_des, std::cout, "jpos_des");

    // Set lowCmd q, dq values
        // Set q des
    cmd.motorCmd[FL_0].q = jpos_des[0];
    cmd.motorCmd[FL_1].q = jpos_des[1];
    cmd.motorCmd[FL_2].q = jpos_des[2];
    cmd.motorCmd[FR_0].q = jpos_des[3];
    cmd.motorCmd[FR_1].q = jpos_des[4];
    cmd.motorCmd[FR_2].q = jpos_des[5];
    cmd.motorCmd[RL_0].q = jpos_des[6];
    cmd.motorCmd[RL_1].q = jpos_des[7];
    cmd.motorCmd[RL_2].q = jpos_des[8];
    cmd.motorCmd[RR_0].q = jpos_des[9];
    cmd.motorCmd[RR_1].q = jpos_des[10];
    cmd.motorCmd[RR_2].q = jpos_des[11];
        // Set q dot des
    cmd.motorCmd[FL_0].dq = command_->qdot[0];
    cmd.motorCmd[FL_1].dq = command_->qdot[1];
    cmd.motorCmd[FL_2].dq = command_->qdot[2];
    cmd.motorCmd[FR_0].dq = command_->qdot[3];
    cmd.motorCmd[FR_1].dq = command_->qdot[4];
    cmd.motorCmd[FR_2].dq = command_->qdot[5];
    cmd.motorCmd[RL_0].dq = command_->qdot[6];
    cmd.motorCmd[RL_1].dq = command_->qdot[7];
    cmd.motorCmd[RL_2].dq = command_->qdot[8];
    cmd.motorCmd[RR_0].dq = command_->qdot[9];
    cmd.motorCmd[RR_1].dq = command_->qdot[10];
    cmd.motorCmd[RR_2].dq = command_->qdot[11];

    // Compute PD Torque value
    // myUtils::pretty_print(kp_, std::cout, "kp");
    // myUtils::pretty_print(kd_, std::cout, "kd");
    // myUtils::pretty_print(jpos_des, std::cout, "jpos_des");
    // myUtils::pretty_print(sensordata_->q, std::cout, "current q");
    for (int i=0; i<12; ++i) {
      // pd_tau[i] = kp_[i] * (jpos_des[i] - sensordata_->q[i]) +
      pd_tau[i] = 
          kp_[i] * (jpos_des[i] - sensordata_->q[i]) +
          kd_[i] * (-sensordata_->qdot[i]);
    }
    // myUtils::pretty_print(pd_tau, std::cout, "pd_tau");


    // Clamp to max/min torque value
    pd_tau[0] = clamp_value(pd_tau[0], -20, 20);
    pd_tau[1] = clamp_value(pd_tau[1], -55, 55);
    pd_tau[2] = clamp_value(pd_tau[2], -55, 55);
    pd_tau[3] = clamp_value(pd_tau[3], -20, 20);
    pd_tau[4]  = clamp_value(pd_tau[4], -55, 55);
    pd_tau[5] = clamp_value(pd_tau[5], -55, 55);
    pd_tau[6] = clamp_value(pd_tau[6], -20, 20);
    pd_tau[7] = clamp_value(pd_tau[7], -55, 55);
    pd_tau[8] = clamp_value(pd_tau[8], -55, 55);
    pd_tau[9] = clamp_value(pd_tau[9], -20, 20);
    pd_tau[10] = clamp_value(pd_tau[10], -55, 55);
    pd_tau[11] = clamp_value(pd_tau[11], -55, 55);

    // myUtils::pretty_print(tau, std::cout, "clamped_tau");

    // Set the torque command
    cmd.motorCmd[FL_0].tau = pd_tau[0];
    cmd.motorCmd[FL_1].tau = pd_tau[1];
    cmd.motorCmd[FL_2].tau = pd_tau[2];
    cmd.motorCmd[FR_0].tau = pd_tau[3];
    cmd.motorCmd[FR_1].tau = pd_tau[4];
    cmd.motorCmd[FR_2].tau = pd_tau[5];
    cmd.motorCmd[RL_0].tau = pd_tau[6];
    cmd.motorCmd[RL_1].tau = pd_tau[7];
    cmd.motorCmd[RL_2].tau = pd_tau[8];
    cmd.motorCmd[RR_0].tau = pd_tau[9];
    cmd.motorCmd[RR_1].tau = pd_tau[10];
    cmd.motorCmd[RR_2].tau = pd_tau[11];
  }

}

void A1LowDriver::set_command() {

  // Do not start controller until we get first sensor data
  if (sensordata_->q.norm() <= 1.) {
    cmd.motorCmd[FL_0].tau = 0.;
    cmd.motorCmd[FL_1].tau = 0.;
    cmd.motorCmd[FL_2].tau = 0.;
    cmd.motorCmd[FR_0].tau = 0.;
    cmd.motorCmd[FR_1].tau = 0.;
    cmd.motorCmd[FR_2].tau = 0.;
    cmd.motorCmd[RL_0].tau = 0.;
    cmd.motorCmd[RL_1].tau = 0.;
    cmd.motorCmd[RL_2].tau = 0.;
    cmd.motorCmd[RR_0].tau = 0.;
    cmd.motorCmd[RR_1].tau = 0.;
    cmd.motorCmd[RR_2].tau = 0.;
  } else {
    // std::cout << "Correct sensor data begins" << std::endl;
    // myUtils::pretty_print(starting_configuration, std::cout, "starting q");
    // myUtils::pretty_print(final_configuration, std::cout, "goal q");
    for(int i=0; i<12; ++i) {
      start_time = curr_time;
      jpos_des[i] =
          myUtils::smooth_changing(starting_configuration[i], final_configuration[i],
                                   5.0, curr_time);
    }
    // myUtils::pretty_print(jpos_des, std::cout, "jpos_des");

    // Set lowCmd q, dq values
        // Set q des
    cmd.motorCmd[FL_0].q = command_->q[0];
    cmd.motorCmd[FL_1].q = command_->q[1];
    cmd.motorCmd[FL_2].q = command_->q[2];
    cmd.motorCmd[FR_0].q = command_->q[3];
    cmd.motorCmd[FR_1].q = command_->q[4];
    cmd.motorCmd[FR_2].q = command_->q[5];
    cmd.motorCmd[RL_0].q = command_->q[6];
    cmd.motorCmd[RL_1].q = command_->q[7];
    cmd.motorCmd[RL_2].q = command_->q[8];
    cmd.motorCmd[RR_0].q = command_->q[9];
    cmd.motorCmd[RR_1].q = command_->q[10];
    cmd.motorCmd[RR_2].q = command_->q[11];
        // Set q dot des
    cmd.motorCmd[FL_0].dq = command_->qdot[0];
    cmd.motorCmd[FL_1].dq = command_->qdot[1];
    cmd.motorCmd[FL_2].dq = command_->qdot[2];
    cmd.motorCmd[FR_0].dq = command_->qdot[3];
    cmd.motorCmd[FR_1].dq = command_->qdot[4];
    cmd.motorCmd[FR_2].dq = command_->qdot[5];
    cmd.motorCmd[RL_0].dq = command_->qdot[6];
    cmd.motorCmd[RL_1].dq = command_->qdot[7];
    cmd.motorCmd[RL_2].dq = command_->qdot[8];
    cmd.motorCmd[RR_0].dq = command_->qdot[9];
    cmd.motorCmd[RR_1].dq = command_->qdot[10];
    cmd.motorCmd[RR_2].dq = command_->qdot[11];

    // Compute PD Torque value
    // myUtils::pretty_print(kp_, std::cout, "kp");
    // myUtils::pretty_print(kd_, std::cout, "kd");
    // myUtils::pretty_print(jpos_des, std::cout, "jpos_des");
    // myUtils::pretty_print(sensordata_->q, std::cout, "current q");
    for (int i=0; i<12; ++i) {
      // pd_tau[i] = kp_[i] * (jpos_des[i] - sensordata_->q[i]) +
      pd_tau[i] = 
          kp_[i] * (command_->q[i] - sensordata_->q[i]) +
          kd_[i] * (command_->qdot[i] - sensordata_->qdot[i]);
    }
    // myUtils::pretty_print(pd_tau, std::cout, "pd_tau");

    // Add PD To FF
    Eigen::VectorXd tau; tau = Eigen::VectorXd::Zero(12);
    for (int i=0; i<12; ++i) {
      tau[i] = command_->jtrq[i] + pd_tau[i];
    }


    // Clamp to max/min torque value
    tau[0] = clamp_value(tau[0], -20, 20);
    tau[1] = clamp_value(tau[1], -55, 55);
    tau[2] = clamp_value(tau[2], -55, 55);
    tau[3] = clamp_value(tau[3], -20, 20);
    tau[4]  = clamp_value(tau[4], -55, 55);
    tau[5] = clamp_value(tau[5], -55, 55);
    tau[6] = clamp_value(tau[6], -20, 20);
    tau[7] = clamp_value(tau[7], -55, 55);
    tau[8] = clamp_value(tau[8], -55, 55);
    tau[9] = clamp_value(tau[9], -20, 20);
    tau[10] = clamp_value(tau[10], -55, 55);
    tau[11] = clamp_value(tau[11], -55, 55);

    // myUtils::pretty_print(tau, std::cout, "clamped_tau");

    // Set the torque command
    cmd.motorCmd[FL_0].tau = tau[0];
    cmd.motorCmd[FL_1].tau = tau[1];
    cmd.motorCmd[FL_2].tau = tau[2];
    cmd.motorCmd[FR_0].tau = tau[3];
    cmd.motorCmd[FR_1].tau = tau[4];
    cmd.motorCmd[FR_2].tau = tau[5];
    cmd.motorCmd[RL_0].tau = tau[6];
    cmd.motorCmd[RL_1].tau = tau[7];
    cmd.motorCmd[RL_2].tau = tau[8];
    cmd.motorCmd[RR_0].tau = tau[9];
    cmd.motorCmd[RR_1].tau = tau[10];
    cmd.motorCmd[RR_2].tau = tau[11];
  }
}

void A1LowDriver::RobotControl() {

  // myUtils::pretty_print(jpos_des, std::cout, "jpos_des");
  curr_time += 0.002;

  // Get the Robot State
  motiontime++;
  udp.GetRecv(state);

  // Get the sensor data into our PnC Objects
  extract_sensor_data();

  // Call A1 Interface getCommand
  interface_->getCommand(sensordata_, command_);

  // Using the PnC Command compute PD vals and set q, qdot, Tau
  set_pd_command();
  // set_command();

  /* interface_->initial_config = starting_configuration;
  interface_->final_config = final_configuration;
  interface_->jpos_step_des = jpos_des;*/

  // Send the command to the robot
  control.PowerProtect(cmd, state, 1);

  udp.SetSend(cmd);

  // if (curr_time >= 0.012) exit(0);
}
