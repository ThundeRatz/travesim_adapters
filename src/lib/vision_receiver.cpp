/**
 * @file vision_receiver.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include "ros/console.h"
#include "gazebo_msgs/ModelStates.h"
#include "travesim_adapters/vision_receiver.h"

VisionReceiver::VisionReceiver() {
    for (int32_t i = 0; i < 7; i++) {
        this->lookup_table.insert({ this->topics[i], i + 1 });
    }
}

int32_t VisionReceiver::model_name_to_index(std::string topic) {
    return this->lookup_table[topic] - 1;
}

void VisionReceiver::receive(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // ROS_INFO_THROTTLE(2, "Received message!");
    // ROS_INFO_STREM("Received message!");
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
        }
    }
}