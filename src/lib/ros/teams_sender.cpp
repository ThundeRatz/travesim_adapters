/**
 * @file teams_transmitter.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief
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
TeamsSender::TeamsSender(ros::NodeHandle* nh) {
    for (uint8_t i = 0; i < NUM_OF_TOPICS_PER_TEAM; i++) {
        this->yellow_pub[i] = nh->advertise<std_msgs::Float64>(this->yellow_topics[i], BUFFER_SIZE);
        this->blue_pub[i] = nh->advertise<std_msgs::Float64>(this->blue_topics[i], BUFFER_SIZE);
        this->yellow_team_cmd[i] = 0;
        this->blue_team_cmd[i] = 0;
    }
}

void TeamsSender::transmit() {
    std_msgs::Float64 yellow_msg;
    std_msgs::Float64 blue_msg;

    for (uint8_t i = 0; i < NUM_OF_TOPICS_PER_TEAM; i++) {
        yellow_msg.data = this->yellow_team_cmd[i];
        blue_msg.data = this->blue_team_cmd[i];

        this->yellow_pub[i].publish(yellow_msg);
        this->blue_pub[i].publish(blue_msg);
    }
}
}  // namespace ros_side
}  // namespace travesim
