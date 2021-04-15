/**
 * @file vision_adapter.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <gazebo_msgs/ModelStates.h>

#include "travesim_adapters/vision_receiver.hpp"
#include "travesim_adapters/protobuf/vision_sender.hpp"
#include "travesim_adapters/data/field_state.hpp"
#include "travesim_adapters/data/robot_state.hpp"
#include "travesim_adapters/data/entity_state.hpp"

#include <iostream>

int main(int argc, char** argv) {
    int32_t send_rate;
    int32_t multicast_port;
    std::string multicast_address_str;

    ros::init(argc, argv, "vision_adapter");
    ros::NodeHandle nh;

    ros::param::param<int32_t>("send_rate", send_rate, 60);

    if (!ros::param::get("vision_multicast_group/port", multicast_port) ||
        !ros::param::get("vision_multicast_group/address", multicast_address_str)) {
        ROS_ERROR_STREAM("Couldn't load multicast port or address!");
        ros::shutdown();
    }

    // ros::param::param<int32_t>("vision_multicast_group/port", multicast_port, 10000);
    // ros::param::param<std::string>("vision_multicast_group/address", multicast_address_str, "224.0.0.4");

    ros::Rate loop_rate(send_rate);

    travesim::VisionReceiver vision_receiver(&nh);
    travesim::proto::VisionSender vision_sender(multicast_address_str, multicast_port);

    travesim::FieldState field_state;

    ROS_INFO_STREAM("Vision adapter started with loop rate " << send_rate);
    ROS_INFO_STREAM("Vision sender multicast addr " << multicast_address_str << ":" << multicast_port);

    while (ros::ok()) {
        field_state.ball = travesim::VisionReceiver::ModelState_to_EntityState(vision_receiver.ball);

        for (uint8_t i = 0; i < 3; i++) {
            field_state.yellow_team[i] =
                travesim::VisionReceiver::ModelState_to_RobotState(vision_receiver.yellow_team[i], true, i);

            field_state.blue_team[i] =
                travesim::VisionReceiver::ModelState_to_RobotState(vision_receiver.blue_team[i], false, i);
        }

        vision_sender.send(&field_state);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
