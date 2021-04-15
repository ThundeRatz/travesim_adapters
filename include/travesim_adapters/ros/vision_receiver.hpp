/**
 * @file vision_receiver.h
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#ifndef __VISION_RECEIVER_H__
#define __VISION_RECEIVER_H__

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

#include <iostream>
#include <unordered_map>

#include "travesim_adapters/ros/teams_topics.hpp"
#include "travesim_adapters/data/data_constants.hpp"
#include "travesim_adapters/data/robot_state.hpp"
#include "travesim_adapters/data/entity_state.hpp"

#define YELLOW_ROBOT_0_NAME "yellow_team/robot_0"
#define YELLOW_ROBOT_1_NAME "yellow_team/robot_1"
#define YELLOW_ROBOT_2_NAME "yellow_team/robot_2"

#define BLUE_ROBOT_0_NAME "blue_team/robot_0"
#define BLUE_ROBOT_1_NAME "blue_team/robot_1"
#define BLUE_ROBOT_2_NAME "blue_team/robot_2"

#define BALL_NAME "vss_ball"

typedef std::unordered_map<std::string, int32_t> lookup_table_t;

namespace travesim {
namespace ros_side {
class VisionReceiver {
    private:
        lookup_table_t lookup_table;
        const std::string topics[NUM_OF_ENTITIES_IN_FIELD] =
        { YELLOW_ROBOT_0_NAME,
          YELLOW_ROBOT_1_NAME,
          YELLOW_ROBOT_2_NAME,
          BLUE_ROBOT_0_NAME,
          BLUE_ROBOT_1_NAME,
          BLUE_ROBOT_2_NAME,
          BALL_NAME };

        ros::Subscriber subscriber;
        gazebo_msgs::ModelState world_state[NUM_OF_ENTITIES_IN_FIELD];

    public:
        VisionReceiver(ros::NodeHandle* nh_ptr);

        gazebo_msgs::ModelState* yellow_team[NUM_OF_ROBOTS_PER_TEAM];
        gazebo_msgs::ModelState* blue_team[NUM_OF_ROBOTS_PER_TEAM];
        gazebo_msgs::ModelState* ball;

        void receive(const gazebo_msgs::ModelStates::ConstPtr& msg);

        int32_t model_name_to_index(std::string topic);

        static travesim::Vector2D Point_to_Vector2D(geometry_msgs::Point* point);

        static travesim::Vector2D Vector3_to_Vector2D(geometry_msgs::Vector3* vector3);

        static travesim::EntityState ModelState_to_EntityState(gazebo_msgs::ModelState* model_state);

        static travesim::RobotState ModelState_to_RobotState(gazebo_msgs::ModelState* model_state,
                                                             bool is_yellow = true, int id = 0);

        static geometry_msgs::Point Vector2D_to_Point(Vector2D* vector2d);

        static geometry_msgs::Vector3 Vector2D_to_Vector3(Vector2D* vector2d);

        static gazebo_msgs::ModelState EntityState_to_ModelState(EntityState* entity_state);

        static gazebo_msgs::ModelState RobotState_to_ModelState(RobotState* robot_state);
};
}  // ros_side
}  // travesim

#endif // __VISION_RECEIVER_H__
