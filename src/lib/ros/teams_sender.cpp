/**
 * @file teams_sender.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief ROS teams commands sender class definition
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "travesim_adapters/ros/teams_sender.hpp"

#define BUFFER_SIZE 2

namespace travesim {
namespace ros_side {
TeamsSender::TeamsSender() {
    ros::NodeHandle nh;

    for (uint8_t i = 0; i < NUM_OF_ROBOTS_PER_TEAM; i++) {
        this->yellow_pub[i] = {.left = nh.advertise<std_msgs::Float64>(this->yellow_topics[2*i], BUFFER_SIZE),
                               .right = nh.advertise<std_msgs::Float64>(this->yellow_topics[2*i+1], BUFFER_SIZE)};

        this->blue_pub[i] = {.left = nh.advertise<std_msgs::Float64>(this->blue_topics[2*i], BUFFER_SIZE),
                             .right = nh.advertise<std_msgs::Float64>(this->blue_topics[2*i+1], BUFFER_SIZE)};
    }
}

void TeamsSender::send(TeamCommand* yellow_team_command, TeamCommand* blue_team_command) {
    std_msgs::Float64 yellow_left_msg, yellow_right_msg;
    std_msgs::Float64 blue_left_msg, blue_right_msg;

    for (uint8_t i = 0; i < NUM_OF_ROBOTS_PER_TEAM; i++) {
        yellow_left_msg.data = yellow_team_command->robot_command[i].left_speed;
        yellow_right_msg.data = yellow_team_command->robot_command[i].right_speed;

        blue_left_msg.data = blue_team_command->robot_command[i].left_speed;
        blue_right_msg.data = blue_team_command->robot_command[i].right_speed;

        this->yellow_pub[i].left.publish(yellow_left_msg);
        this->yellow_pub[i].right.publish(yellow_right_msg);

        this->blue_pub[i].left.publish(blue_left_msg);
        this->blue_pub[i].right.publish(blue_right_msg);
    }
}
}  // namespace ros_side
}  // namespace travesim
