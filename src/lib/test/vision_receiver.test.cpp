// Bring in my package's API, which is what I'm testing
#include "ros/ros.h"
#include "ros/console.h"
#include "travesim_adapters/vision_receiver.hpp"
#include <gtest/gtest.h>

// Declare a test
TEST(vision_adapter, model_name_to_index)
{
    ros::NodeHandle _nh;
    VisionReceiver vision_receiver(&_nh);

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
