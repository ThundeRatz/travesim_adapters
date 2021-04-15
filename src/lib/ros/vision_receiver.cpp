/**
 * @file vision_receiver.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include <ros/console.h>
#include <gazebo_msgs/ModelStates.h>
#include "travesim_adapters/ros/vision_receiver.hpp"

#define DEFAULT_Z_VALUE 0.022
#define DEFAULT_ENTITY_NAME "vss_ball"
#define DEFAULT_REFERENCE_FRAME "world"

#define quaternion_to_theta(q0, q1, q2, q3) atan2(2*(q0*q1+q2*q3), 1 - 2*(q1*q1 + q2*q2))

namespace travesim {
namespace ros_side {
VisionReceiver::VisionReceiver(ros::NodeHandle* nh_ptr) {
    for (int32_t i = 0; i < NUM_OF_ENTITIES_IN_FIELD; i++) {
        this->lookup_table.insert({ this->topics[i], i + 1 });
    }

    for (int32_t i = 0; i < NUM_OF_ROBOTS_PER_TEAM; i++) {
        this->yellow_team[i] = &this->world_state[i];
        this->blue_team[i] = &this->world_state[i + NUM_OF_ROBOTS_PER_TEAM];
    }

    this->ball = &this->world_state[NUM_OF_ENTITIES_IN_FIELD - 1];

    this->received_first_message = false;

    this->subscriber = nh_ptr->subscribe("/gazebo/model_states", 2, &VisionReceiver::receive, this);
}

int32_t VisionReceiver::model_name_to_index(std::string model_name) {
    return this->lookup_table[model_name] - 1;
}

bool VisionReceiver::get_received_first_message() {
    return this->received_first_message;
}

void VisionReceiver::receive(const gazebo_msgs::ModelStates::ConstPtr& msg) {
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

            this->received_first_message = true;
        }
    }
}

Vector2D VisionReceiver::Point_to_Vector2D(geometry_msgs::Point* point) {
    Vector2D retval;

    retval.x = point->x;
    retval.y = point->y;

    return retval;
}

Vector2D VisionReceiver::Vector3_to_Vector2D(geometry_msgs::Vector3* vector3) {
    Vector2D retval;

    retval.x = vector3->x;
    retval.y = vector3->y;

    return retval;
}

geometry_msgs::Vector3 VisionReceiver::Vector2D_to_Vector3(Vector2D* vector2d) {
    geometry_msgs::Vector3 retval;

    retval.x = vector2d->x;
    retval.y = vector2d->y;
    retval.z = 0;

    return retval;
}

geometry_msgs::Point VisionReceiver::Vector2D_to_Point(Vector2D* vector2d) {
    geometry_msgs::Point retval;

    retval.x = vector2d->x;
    retval.y = vector2d->y;
    retval.z = DEFAULT_Z_VALUE;

    return retval;
}

EntityState VisionReceiver::ModelState_to_EntityState(gazebo_msgs::ModelState* model_state) {
    EntityState retval;

    retval.position = Point_to_Vector2D(&model_state->pose.position);

    retval.angular_position = quaternion_to_theta(model_state->pose.orientation.w, model_state->pose.orientation.x,
                                                  model_state->pose.orientation.y, model_state->pose.orientation.z);

    retval.velocity = Vector3_to_Vector2D(&model_state->twist.linear);

    retval.angular_velocity = model_state->twist.angular.z;

    return retval;
}

RobotState VisionReceiver::ModelState_to_RobotState(gazebo_msgs::ModelState* model_state, bool is_yellow, int id) {
    EntityState tmp = ModelState_to_EntityState(model_state);

    RobotState retval(&tmp, is_yellow, id);

    return retval;
}

gazebo_msgs::ModelState VisionReceiver::EntityState_to_ModelState(EntityState* entity_state) {
    gazebo_msgs::ModelState retval;

    retval.model_name = DEFAULT_ENTITY_NAME;
    retval.reference_frame = DEFAULT_REFERENCE_FRAME;

    retval.pose.position = Vector2D_to_Point(&entity_state->position);

    retval.pose.orientation.w = cos(entity_state->angular_position/2);
    retval.pose.orientation.x = sin(entity_state->angular_position/2);
    retval.pose.orientation.y = 0;
    retval.pose.orientation.z = 0;

    retval.twist.linear = Vector2D_to_Vector3(&entity_state->velocity);

    retval.twist.angular.x = 0;
    retval.twist.angular.y = 0;
    retval.twist.angular.z = entity_state->angular_velocity;

    return retval;
}

gazebo_msgs::ModelState VisionReceiver::RobotState_to_ModelState(RobotState* robot_state) {
    gazebo_msgs::ModelState retval = EntityState_to_ModelState(dynamic_cast<EntityState*>(robot_state));

    std::string base_name = robot_state->is_yellow ? "yellow_team/robot_" : "blue_team/robot_";
    retval.model_name = base_name.append(std::to_string(robot_state->id));

    return retval;
}
}  // namespace ros_side
}  // namespace travesim
