/**
 * @file vision_receiver.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief ROS vision receiver class definition
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include <ros/console.h>
#include <gazebo_msgs/ModelStates.h>
#include "travesim_adapters/ros/vision_receiver.hpp"

namespace travesim {
namespace ros_side {
VisionReceiver::VisionReceiver() {
    this->received_first_message = false;

    ros::NodeHandle _nh;
    this->subscriber = _nh.subscribe("/gazebo/model_states", 2, &VisionReceiver::receive, this);
}

bool VisionReceiver::get_received_first_message() {
    return this->received_first_message;
}

void VisionReceiver::receive(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    this->received_first_message = true;
    this->world_state = msg;
}
}  // namespace ros_side
}  // namespace travesim
