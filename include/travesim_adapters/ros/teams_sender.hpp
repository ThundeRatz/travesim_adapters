/**
 * @file teams_sender.hpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief ROS teams commands sender class definition
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#ifndef __TEAMS_SENDER_H__
#define __TEAMS_SENDER_H__

#include <ros/ros.h>
#include <ros/console.h>
#include "travesim_adapters/ros/teams_topics.hpp"
#include "travesim_adapters/data/data_constants.hpp"

#include <iostream>

namespace travesim {
namespace ros_side {
class TeamsSender {
    private:
        /**
         * @brief Publishers to Gazebo controllers
         *
         */
        ros::Publisher yellow_pub[NUM_OF_COMMANDS_PER_TEAM];
        ros::Publisher blue_pub[NUM_OF_COMMANDS_PER_TEAM];

        /**
         * @brief List of yellow team's topics names
         *
         */
        const std::string yellow_topics[NUM_OF_COMMANDS_PER_TEAM] = {
            YELLOW_ROBOT_0_LEFT_TOPIC, YELLOW_ROBOT_0_RIGHT_TOPIC,
            YELLOW_ROBOT_1_LEFT_TOPIC, YELLOW_ROBOT_1_RIGHT_TOPIC,
            YELLOW_ROBOT_2_LEFT_TOPIC, YELLOW_ROBOT_2_RIGHT_TOPIC };

        /**
         * @brief List of blue team's topics names
         *
         */
        const std::string blue_topics[NUM_OF_COMMANDS_PER_TEAM] = {
            BLUE_ROBOT_0_LEFT_TOPIC, BLUE_ROBOT_0_RIGHT_TOPIC,
            BLUE_ROBOT_1_LEFT_TOPIC, BLUE_ROBOT_1_RIGHT_TOPIC,
            BLUE_ROBOT_2_LEFT_TOPIC, BLUE_ROBOT_2_RIGHT_TOPIC };

    public:
        /**
         * @brief Cache of yellow team's commands
         *
         */
        double yellow_team_cmd[NUM_OF_COMMANDS_PER_TEAM];

        /**
         * @brief Cache of blue team's commands
         *
         */
        double blue_team_cmd[NUM_OF_COMMANDS_PER_TEAM];

        /**
         * @brief Construct a new Teams Sender object
         *
         */
        TeamsSender();

        /**
         * @brief Send the stored commands to Gazebo controllers
         *
         */
        void transmit();

        /**
         * @brief Funcion to be called when a protobuf message arives
         *
         */
        void protobuf_callback();
};
}  // ros_side
}  // travesim

#endif // __TEAMS_SENDER_H__
