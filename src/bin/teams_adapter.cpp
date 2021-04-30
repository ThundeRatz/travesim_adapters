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

#include "travesim_adapters/data/team_command.hpp"
#include "travesim_adapters/protobuf/team_receiver.hpp"

#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "teams_adapter");
    ros::NodeHandle nh;

    int32_t send_rate;
    int32_t yellow_port, blue_port;
    std::string yellow_address_str, blue_address_str;

    ros::param::param<int32_t>("send_rate", send_rate, 60);

    if (!ros::param::get("teams_control_client/yellow/port", yellow_port) ||
        !ros::param::get("teams_control_client/yellow/address", yellow_address_str)) {
        ROS_ERROR_STREAM("Couldn't load yellow team port or address!");
        ros::shutdown();
    }

    if (!ros::param::get("teams_control_client/blue/port", blue_port) ||
        !ros::param::get("teams_control_client/blue/address", blue_address_str)) {
        ROS_ERROR_STREAM("Couldn't load blue team port or address!");
        ros::shutdown();
    }

    travesim::proto::TeamReceiver yellow_receiver(yellow_address_str, yellow_port, true);
    travesim::proto::TeamReceiver blue_receiver(blue_address_str, blue_port, false);

    travesim::ros_side::TeamsSender teams_sender;

    travesim::TeamCommand yellow_command, blue_command;

    ros::Rate loop_rate(send_rate);

    ROS_INFO_STREAM("Teams adapter started with loop rate " << send_rate);

    while (ros::ok()) {
        yellow_receiver.receive(&yellow_command);
        blue_receiver.receive(&blue_command);

        teams_sender.send(&yellow_command, &blue_command);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
