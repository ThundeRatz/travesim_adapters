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

#include "travesim_adapters/ros/vision_receiver.hpp"

TEST(vision_receiver, model_name_to_index)
{
    travesim::ros_side::VisionReceiver vision_receiver;

    EXPECT_EQ(vision_receiver.model_name_to_index("yellow_team/robot_0"), 0);
    EXPECT_EQ(vision_receiver.model_name_to_index("yellow_team/robot_1"), 1);
    EXPECT_EQ(vision_receiver.model_name_to_index("yellow_team/robot_2"), 2);

    EXPECT_EQ(vision_receiver.model_name_to_index("blue_team/robot_0"), 3);
    EXPECT_EQ(vision_receiver.model_name_to_index("blue_team/robot_1"), 4);
    EXPECT_EQ(vision_receiver.model_name_to_index("blue_team/robot_2"), 5);

    EXPECT_EQ(vision_receiver.model_name_to_index("vss_ball"), 6);

    EXPECT_EQ(vision_receiver.model_name_to_index("invalid/name"), -1);
    EXPECT_EQ(vision_receiver.model_name_to_index("another/invalid/name"), -1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
