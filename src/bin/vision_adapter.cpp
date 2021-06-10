/**
 * @file vision_adapter.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 * @brief Vision adapter executable file
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 * @see VisionReceiver
 * @see VisionSender
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <gazebo_msgs/ModelStates.h>

#include "travesim_adapters/ros/vision_receiver.hpp"
#include "travesim_adapters/data/converter/ros_side.hpp"
#include "travesim_adapters/data/field_state.hpp"
#include "travesim_adapters/data/robot_state.hpp"
#include "travesim_adapters/data/entity_state.hpp"
#include "travesim_adapters/protobuf/vision_sender.hpp"

#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_adapter");
    ros::NodeHandle nh;

    int32_t multicast_port;
    std::string multicast_address_str;

    if (!ros::param::get("vision_multicast_group/port", multicast_port) ||
        !ros::param::get("vision_multicast_group/address", multicast_address_str)) {
        ROS_ERROR_STREAM("Couldn't load multicast port or address!");
        ros::shutdown();
    }

    travesim::ros_side::VisionReceiver vision_receiver;
    travesim::proto::VisionSender vision_sender(multicast_address_str, multicast_port);

    travesim::FieldState field_state;
    gazebo_msgs::ModelStates::ConstPtr world_state;

    ROS_INFO_STREAM("Vision sender multicast endpoint " << multicast_address_str << ":" << multicast_port);

    field_state.time_step = 0;

    while (ros::ok()) {
        if (vision_receiver.receive(&world_state)) {
            field_state = travesim::converter::ModelStates_to_FieldState(world_state);
            vision_sender.send(&field_state);
            field_state.time_step++;
        }

        ros::spinOnce();
    }
}
