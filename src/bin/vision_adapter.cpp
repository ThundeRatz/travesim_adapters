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
#include "travesim_adapters/data/field_state.hpp"
#include "travesim_adapters/data/robot_state.hpp"
#include "travesim_adapters/data/entity_state.hpp"

#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_adapter");
    ros::NodeHandle nh;

    int32_t send_rate;
    travesim::VisionReceiver vision_receiver(&nh);

    travesim::FieldState field_state;
    nh.param<int32_t>("send_rate", send_rate, 60);
    ros::Rate loop_rate(send_rate);

    ROS_INFO_STREAM("Vision adapter started with loop rate " << send_rate);

    while (ros::ok()) {
        // Send message with protobuf

        field_state.ball = travesim::VisionReceiver::ModelState_to_EntityState(vision_receiver.ball);

        for (uint8_t i = 0; i < 3; i++) {
            field_state.yellow_team[i] =
                travesim::VisionReceiver::ModelState_to_RobotState(vision_receiver.yellow_team[i], true, i);

            field_state.blue_team[i] =
                travesim::VisionReceiver::ModelState_to_RobotState(vision_receiver.blue_team[i], false, i);
        }

        ROS_INFO_STREAM(field_state);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
