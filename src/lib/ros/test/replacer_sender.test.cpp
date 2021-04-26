/**
 * @file vision_receiver.test.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief Tests for vision_receiver.cpp
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 * @see VisionReceiver
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <gtest/gtest.h>

#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>

#include "travesim_adapters/ros/replacer_sender.hpp"

TEST(replacer_sender, msg_received)
{
    ros::NodeHandle _nh;
    ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states", _nh);

    travesim::ros_side::ReplacerSender replacer_sender;

    std::vector<gazebo_msgs::ModelStatePtr> list_model_states;

    list_model_states.insert(list_model_states.begin(), gazebo_msgs::ModelStatePtr(new gazebo_msgs::ModelState()));

    list_model_states[0]->model_name = "yellow_team/robot_0";
    list_model_states[0]->pose.position.x = 1.0;
    list_model_states[0]->pose.position.y = 2.0;

    EXPECT_TRUE(replacer_sender.set_team_state(&list_model_states));

    gazebo_msgs::GetModelState get_model_state;
    get_model_state.request.model_name = "yellow_team/robot_0";
    get_model_state.request.relative_entity_name = "world";

    ros::service::call("/gazebo/get_model_state", get_model_state);

    EXPECT_TRUE(get_model_state.response.success);

    EXPECT_LT(abs(get_model_state.response.pose.position.x - 1.0), 0.001);
    EXPECT_LT(abs(get_model_state.response.pose.position.y - 2.0), 0.001);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester_replacer_sender");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
