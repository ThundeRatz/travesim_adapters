/**
 * @file robot_state.test.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <gtest/gtest.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>

#include "travesim_adapters/data/robot_state.hpp"
#include "travesim_adapters/ros/vision_receiver.hpp"

// Declare a test
TEST(robot_state_to_model_state, model_name)
{
    travesim::RobotState robot_state;

    EXPECT_EQ(travesim::ros_side::VisionReceiver::RobotState_to_ModelState(
                  &robot_state).model_name, "yellow_team/robot_0");

    robot_state.id = 2;

    EXPECT_EQ(travesim::ros_side::VisionReceiver::RobotState_to_ModelState(
                  &robot_state).model_name, "yellow_team/robot_2");

    robot_state.is_yellow = false;
    robot_state.id = 0;

    EXPECT_EQ(travesim::ros_side::VisionReceiver::RobotState_to_ModelState(&robot_state).model_name,
              "blue_team/robot_0");

    robot_state.id = 1;

    EXPECT_EQ(travesim::ros_side::VisionReceiver::RobotState_to_ModelState(&robot_state).model_name,
              "blue_team/robot_1");
}

TEST(robot_state_to_model_state, angle_conversion)
{
    travesim::RobotState robot_state;
    robot_state.angular_position = -M_PI/3;

    gazebo_msgs::ModelState model_state = travesim::ros_side::VisionReceiver::RobotState_to_ModelState(&robot_state);

    EXPECT_DOUBLE_EQ(travesim::ros_side::VisionReceiver::ModelState_to_RobotState(&model_state).angular_position,
                     robot_state.angular_position);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
