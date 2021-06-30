/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

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

    Control control;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};

void Custom::UDPRecv()
{ 
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime++;
    udp.GetRecv(state);


    // ----------------------------------------------- //
    // Check if we want to use footForce or footForceEst
    std::cout << "FL_foot force = " << state.footForce[0] << "   " <<state.footForceEst[0] << std::endl;
    // ----------------------------------------------- //

    // ----------------------------------------------- //
    // Print Data
    std::cout << "FL_foot force = " << state.footForce[0] << std::endl;
    std::cout << "FR_foot force = " << state.footForce[1] << std::endl;
    std::cout << "RL_foot force = " << state.footForce[2] << std::endl;
    std::cout << "RR_foot force = " << state.footForce[3] << std::endl;
    // ----------------------------------------------- //


    // printf("%d\n", motiontime);
    // gravity compensation
    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

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
