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
#include "travesim_adapters/data/team_command.hpp"
#include "travesim_adapters/data/data_common.hpp"

#include <iostream>

namespace travesim {
namespace ros_side {
/**
 * @brief Helper struct to group same robot command topics
 */
typedef struct {
    ros::Publisher left;
    ros::Publisher right;
} robot_command_pub_t;

class TeamsSender {
    private:
        /**
         * @brief Yellow team ROS publishers
         *
         */
        robot_command_pub_t yellow_pub[NUM_OF_ROBOTS_PER_TEAM];

        /**
         * @brief Blue team ROS publishers
         *
         */
        robot_command_pub_t blue_pub[NUM_OF_ROBOTS_PER_TEAM];

        /**
         * @brief List of yellow team's topics names
         *
         */
        const std::string yellow_topics[2*NUM_OF_ROBOTS_PER_TEAM] = {
            YELLOW_ROBOT_0_LEFT_TOPIC, YELLOW_ROBOT_0_RIGHT_TOPIC,
            YELLOW_ROBOT_1_LEFT_TOPIC, YELLOW_ROBOT_1_RIGHT_TOPIC,
            YELLOW_ROBOT_2_LEFT_TOPIC, YELLOW_ROBOT_2_RIGHT_TOPIC };

        /**
         * @brief List of blue team's topics names
         *
         */
        const std::string blue_topics[2*NUM_OF_ROBOTS_PER_TEAM] = {
            BLUE_ROBOT_0_LEFT_TOPIC, BLUE_ROBOT_0_RIGHT_TOPIC,
            BLUE_ROBOT_1_LEFT_TOPIC, BLUE_ROBOT_1_RIGHT_TOPIC,
            BLUE_ROBOT_2_LEFT_TOPIC, BLUE_ROBOT_2_RIGHT_TOPIC };

    public:
        /**
         * @brief Construct a new Teams Sender object
         *
         */
        TeamsSender();

        /**
         * @brief Send the stored commands to Gazebo controllers
         *
         */
        void send(TeamCommand* yellow_team_command, TeamCommand* blue_team_command);
};
}  // ros_side
}  // travesim

#endif // __TEAMS_SENDER_H__
