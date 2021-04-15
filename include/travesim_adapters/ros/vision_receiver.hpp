/**
 * @file vision_receiver.h
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#ifndef __VISION_RECEIVER_H__
#define __VISION_RECEIVER_H__

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

#include <iostream>
#include <unordered_map>

#include "travesim_adapters/ros/teams_topics.hpp"
#include "travesim_adapters/data/data_constants.hpp"
#include "travesim_adapters/data/robot_state.hpp"
#include "travesim_adapters/data/entity_state.hpp"

#define YELLOW_ROBOT_0_NAME "yellow_team/robot_0"
#define YELLOW_ROBOT_1_NAME "yellow_team/robot_1"
#define YELLOW_ROBOT_2_NAME "yellow_team/robot_2"

#define BLUE_ROBOT_0_NAME "blue_team/robot_0"
#define BLUE_ROBOT_1_NAME "blue_team/robot_1"
#define BLUE_ROBOT_2_NAME "blue_team/robot_2"

#define BALL_NAME "vss_ball"

typedef std::unordered_map<std::string, int32_t> lookup_table_t;

namespace travesim {
namespace ros_side {
/**
 * @brief Class to receive Gazebo published messages
 *
 */
class VisionReceiver {
    private:
        /**
         * @brief Lookup table to map
         *  topic name -> position in message array
         *
         * @note This table is not intended for direct use,
         * please refer to model_name_to_index() instead
         *
         * @see model_name_to_index
         */
        lookup_table_t lookup_table;

        /**
         * @brief List of model names
         *  This is the inverse function from lookup table
         */
        const std::string topics[NUM_OF_ENTITIES_IN_FIELD] =
        { YELLOW_ROBOT_0_NAME,
          YELLOW_ROBOT_1_NAME,
          YELLOW_ROBOT_2_NAME,
          BLUE_ROBOT_0_NAME,
          BLUE_ROBOT_1_NAME,
          BLUE_ROBOT_2_NAME,
          BALL_NAME };

        /**
         * @brief Subscriber for Gazebo model_states topic
         */
        ros::Subscriber subscriber;

        /**
         * @brief Cache to save the world state
         */
        gazebo_msgs::ModelState world_state[NUM_OF_ENTITIES_IN_FIELD];

        /**
         * @brief Flag to prevent message sending before
         * the first message arives
         */
        bool received_first_message;

    public:
        /**
         * @brief Helper pointers to world state cache
         */
        gazebo_msgs::ModelState* yellow_team[NUM_OF_ROBOTS_PER_TEAM];
        gazebo_msgs::ModelState* blue_team[NUM_OF_ROBOTS_PER_TEAM];
        gazebo_msgs::ModelState* ball;

        /**
         * @brief Construct a new Vision Receiver object
         *
         * @param nh_ptr NodeHandle pointer to create subscribers
         */
        VisionReceiver(ros::NodeHandle* nh_ptr);

        /**
         * @brief Callback function for Gazebo subscriber
         *
         * @param msg
         */
        void receive(const gazebo_msgs::ModelStates::ConstPtr& msg);

        /**
         * @brief Helper function to map simulation model names
         * to array positions
         *
         * @param model_name Name of the desired model
         * @return int32_t Index whitin world state array.
         *  Return -1 if invalid name is given
         */
        int32_t model_name_to_index(std::string model_name);

        /**
         * @brief Get the received first message flag
         *
         * @return true If a message have already arrived
         * @return false Othwerwise
         */
        bool get_received_first_message();

        /**
         * @brief Static function to convert geometry_msgs::Point to travesim::Vector2D
         *
         * @param point Data to be converted
         * @return travesim::Vector2D Converted data
         */
        static travesim::Vector2D Point_to_Vector2D(geometry_msgs::Point* point);

        /**
         * @brief Static function to convert geometry_msgs::Vector3 to travesim::Vector2D
         *
         * @param vector3 Data to be converted
         * @return travesim::Vector2D Converted data
         */
        static travesim::Vector2D Vector3_to_Vector2D(geometry_msgs::Vector3* vector3);

        /**
         * @brief Static function to convert gazebo_msgs::ModelState to travesim::EntityState
         *
         * @param model_state Data to be converted
         * @return travesim::EntityState Converted data
         */
        static travesim::EntityState ModelState_to_EntityState(gazebo_msgs::ModelState* model_state);

        /**
         * @brief Static function to convert gazebo_msgs::ModelState to travesim::RobotState
         *
         * @param model_state Data to be converted
         * @param is_yellow True if the robot is from the yellow team, false otherwise
         * @param id Id number of the robot
         * @return travesim::RobotState Converte data
         */
        static travesim::RobotState ModelState_to_RobotState(gazebo_msgs::ModelState* model_state,
                                                             bool is_yellow = true, int id = 0);

        /**
         * @brief Static function to convert travesim::Vector2D to geometry_msgs::Point
         *
         * @param vector2d Data to be converted
         * @return geometry_msgs::Point Converted data
         */
        static geometry_msgs::Point Vector2D_to_Point(Vector2D* vector2d);

        /**
         * @brief Static function to convert travesim::Vector2D to geometry_msgs::Vector3
         *
         * @param vector2d Data to be converted
         * @return geometry_msgs::Vector3 Converted data
         */
        static geometry_msgs::Vector3 Vector2D_to_Vector3(Vector2D* vector2d);

        /**
         * @brief Static function to convert travesim::EntityState to gazebo_msgs::ModelState
         *
         * @param entity_state Data to be converted
         * @return gazebo_msgs::ModelState Converted data
         */
        static gazebo_msgs::ModelState EntityState_to_ModelState(EntityState* entity_state);

        /**
         * @brief Static function to convert travesim::RobotState to gazebo_msgs::ModelState
         *
         * @param entity_state Data to be converted
         * @return gazebo_msgs::ModelState Converted data
         */
        static gazebo_msgs::ModelState RobotState_to_ModelState(RobotState* robot_state);
};
}  // ros_side
}  // travesim

#endif // __VISION_RECEIVER_H__
