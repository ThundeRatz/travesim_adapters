/**
 * @file vision_receiver.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief
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
    for (int32_t i = 0; i < NUM_OF_ENTITIES_IN_FIELD; i++) {
        this->lookup_table.insert({ this->topics[i], i + 1 });
    }

    for (int32_t i = 0; i < NUM_OF_ROBOTS_PER_TEAM; i++) {
        this->yellow_team[i] = &this->world_state[i];
        this->blue_team[i] = &this->world_state[i + NUM_OF_ROBOTS_PER_TEAM];
    }

    this->ball = &this->world_state[NUM_OF_ENTITIES_IN_FIELD - 1];

    this->received_first_message = false;

    ros::NodeHandle _nh;
    this->subscriber = _nh.subscribe("/gazebo/model_states", 2, &VisionReceiver::receive, this);
}

int32_t VisionReceiver::model_name_to_index(std::string model_name) {
    return this->lookup_table[model_name] - 1;
}

bool VisionReceiver::get_received_first_message() {
    return this->received_first_message;
}

void VisionReceiver::receive(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    int32_t index = -1;
    uint32_t size = msg->name.size();

    for (uint32_t i = 0; i < size; i++) {
        index = this->model_name_to_index(msg->name[i]);

        if (index != -1) {
            ROS_DEBUG_STREAM(msg->name[i] << " X: " << msg->pose[i].position.x << " Y: " << msg->pose[i].position.y);
            this->world_state[index].pose = msg->pose[i];
            this->world_state[index].twist = msg->twist[i];
            this->world_state[index].model_name = msg->name[i];
            this->world_state[index].reference_frame = "world";

            this->received_first_message = true;
        }
    }
}
}  // namespace ros_side
}  // namespace travesim
