/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

#include <Configuration.h>
#include <PnC/A1PnC/A1Interface.hpp>
#include <Utils/IO/IOUtilities.hpp>

#include <Eigen/Dense>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(): control(LeggedType::A1, LOWLEVEL), udp(){
        control.InitCmdData(cmd);
    }
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

    Eigen::VectorXd kp_;
    Eigen::VectorXd kd_;
    Eigen::VectorXd initial_configuration;
};



void Custom::InitializePnC() {
    // interface_ = new A1Interface();
    sensordata_ = new A1SensorData();
    command_ = new A1Command();
    kp_ = Eigen::VectorXd::Zero(12);
    kd_ = Eigen::VectorXd::Zero(12);
    initial_configuration = Eigen::VectorXd::Zero(12);

    try {
    YAML::Node simulation_cfg =
        YAML::LoadFile("/home/knapsack/project/locomotion/src/PnC/Config/A1/TEST/REAL.yaml");
    myUtils::readParameter(simulation_cfg, "initial_configuration", initial_configuration);
    YAML::Node control_cfg = simulation_cfg["control_configuration"];
    myUtils::readParameter(control_cfg, "kp", kp_);
    myUtils::readParameter(control_cfg, "kd", kd_);
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }

}

void Custom::UDPRecv()
{ 
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() {
    // Get the Robot State
    motiontime++;
    udp.GetRecv(state);

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
    myUtils::pretty_print(sensordata_->q, std::cout, "Current q");
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
    myUtils::pretty_print(sensordata_->qdot, std::cout, "Current qdot");
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
        // Send the command to the robot
    control.PowerProtect(cmd, state, 1);

    udp.SetSend(cmd);
}

int main(void)
{
    std::cout << "Control level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom;
    custom.InitializePnC();

    
    
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
