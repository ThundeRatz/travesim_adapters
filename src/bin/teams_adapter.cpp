/**
 * @file teams_adapter.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief Teams commands adapter execution file
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelStates.h>
#include "travesim_adapters/ros/teams_topics.hpp"
#include "travesim_adapters/ros/teams_sender.hpp"

#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "teams_adapter");
    ros::NodeHandle nh;

    int32_t send_rate;
    travesim::ros_side::TeamsSender teams_sender(&nh);

    nh.param<int32_t>("send_rate", send_rate, 60);

    ros::Rate loop_rate(send_rate);

    ROS_INFO_STREAM("Teams adapter started with loop rate " << send_rate);

    while (ros::ok()) {
        teams_sender.transmit();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
