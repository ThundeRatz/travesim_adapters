/**
 * @file vision_receiver.hpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief ROS vision receiver class definition
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

#include "travesim_adapters/ros/teams_topics.hpp"
#include "travesim_adapters/data/data_constants.hpp"
#include "travesim_adapters/data/robot_state.hpp"
#include "travesim_adapters/data/entity_state.hpp"

namespace travesim {
namespace ros_side {
/**
 * @brief Class to receive Gazebo published messages
 *
 */
class VisionReceiver {
    private:
        /**
         * @brief Subscriber for Gazebo model_states topic
         */
        ros::Subscriber subscriber;

        /**
         * @brief Flag to prevent message sending before
         * the first message arives
         */
        bool received_first_message;

    public:
        /**
         * @brief Cache to save the world state
         */
        gazebo_msgs::ModelStates::ConstPtr world_state;

        /**
         * @brief Construct a new Vision Receiver object
         *
         * @note Must be called after ros::init()!
         */
        VisionReceiver();

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
};
}  // ros_side
}  // travesim

#endif // __VISION_RECEIVER_H__
