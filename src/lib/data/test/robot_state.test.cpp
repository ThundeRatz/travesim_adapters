// Bring in my package's API, which is what I'm testing
#include <ros/ros.h>
#include <ros/console.h>
#include <gtest/gtest.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Vector3.h>

#include "travesim_adapters/data/robot_state.hpp"

// Declare a test
TEST(robot_state_to_model_state, model_name)
{
    travesim::RobotState robot_state;

    EXPECT_EQ(robot_state.to_ModelState().model_name, "yellow_team/robot_0");

    robot_state.id = 2;

    EXPECT_EQ(robot_state.to_ModelState().model_name, "yellow_team/robot_2");

    robot_state.is_yellow = false;
    robot_state.id = 0;

    EXPECT_EQ(robot_state.to_ModelState().model_name, "blue_team/robot_0");

    robot_state.id = 1;

    EXPECT_EQ(robot_state.to_ModelState().model_name, "blue_team/robot_1");
}

TEST(robot_state_to_model_state, angle_conversion)
{
    travesim::RobotState robot_state;
    robot_state.angular_position = -M_PI/3;

    gazebo_msgs::ModelState model_state = robot_state.to_ModelState();

    EXPECT_DOUBLE_EQ(travesim::RobotState(&model_state).angular_position, robot_state.angular_position);
}

TEST(vector2d_to_vector3, convert_from_vector2d)
{
    travesim::Vector2D vector2d(2, 4);

    geometry_msgs::Vector3 vector3 = vector2d.to_Vector3();

    EXPECT_DOUBLE_EQ(vector3.x, 2);
    EXPECT_DOUBLE_EQ(vector3.y, 4);
}

TEST(vector2d_to_vector3, convert_from_vector3)
{
    geometry_msgs::Vector3 vector3;

    vector3.x = 3;
    vector3.y = -7;

    travesim::Vector2D vector2d(&vector3);

    EXPECT_DOUBLE_EQ(vector2d.x, 3);
    EXPECT_DOUBLE_EQ(vector2d.y, -7);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
