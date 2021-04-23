/**
 * @file converter.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief Collection of data converters between ROS and local formats
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include "travesim_adapters/data/converter/ros_side.hpp"

#define quaternion_to_theta(q0, q1, q2, q3) atan2(2*(q0*q1+q2*q3), 1 - 2*(q1*q1 + q2*q2))

typedef std::unordered_map<std::string, travesim::EntityState*> lookup_table_t;

namespace travesim {
namespace converter {
Vector2D Point_to_Vector2D(geometry_msgs::Point* point) {
    Vector2D retval;

    retval.x = point->x;
    retval.y = point->y;

    return retval;
}

Vector2D Vector3_to_Vector2D(geometry_msgs::Vector3* vector3) {
    Vector2D retval;

    retval.x = vector3->x;
    retval.y = vector3->y;

    return retval;
}

geometry_msgs::Vector3 Vector2D_to_Vector3(Vector2D* vector2d) {
    geometry_msgs::Vector3 retval;

    retval.x = vector2d->x;
    retval.y = vector2d->y;
    retval.z = 0;

    return retval;
}

geometry_msgs::Point Vector2D_to_Point(Vector2D* vector2d, double z) {
    geometry_msgs::Point retval;

    retval.x = vector2d->x;
    retval.y = vector2d->y;
    retval.z = z;

    return retval;
}

EntityState ModelState_to_EntityState(gazebo_msgs::ModelState* model_state) {
    EntityState retval;

    retval.position = Point_to_Vector2D(&model_state->pose.position);

    retval.angular_position = quaternion_to_theta(model_state->pose.orientation.w, model_state->pose.orientation.x,
                                                  model_state->pose.orientation.y, model_state->pose.orientation.z);

    retval.velocity = Vector3_to_Vector2D(&model_state->twist.linear);

    retval.angular_velocity = model_state->twist.angular.z;

    return retval;
}

RobotState ModelState_to_RobotState(gazebo_msgs::ModelState* model_state, bool is_yellow, int id) {
    EntityState tmp = ModelState_to_EntityState(model_state);

    RobotState retval(&tmp, is_yellow, id);

    return retval;
}

gazebo_msgs::ModelState EntityState_to_ModelState(EntityState* entity_state, double z) {
    gazebo_msgs::ModelState retval;

    retval.model_name = DEFAULT_ENTITY_NAME;
    retval.reference_frame = DEFAULT_REFERENCE_FRAME;

    retval.pose.position = Vector2D_to_Point(&entity_state->position, z);

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

gazebo_msgs::ModelState RobotState_to_ModelState(RobotState* robot_state, double z) {
    gazebo_msgs::ModelState retval = EntityState_to_ModelState(dynamic_cast<EntityState*>(robot_state), z);

    std::string base_name = robot_state->is_yellow ? "yellow_team/robot_" : "blue_team/robot_";
    retval.model_name = base_name.append(std::to_string(robot_state->id));

    return retval;
}

FieldState ModelStates_to_FieldState(gazebo_msgs::ModelStates::ConstPtr model_states) {
    static FieldState field_state;

    /**
     * @brief This lookup table will map model_name -> field_state data location,
     * so we can acess field_state elements from their's names
     *
     */
    static lookup_table_t lookup_table({{YELLOW_ROBOT_0_NAME, &field_state.yellow_team[0]},
                                           {YELLOW_ROBOT_1_NAME, &field_state.yellow_team[1]},
                                           {YELLOW_ROBOT_2_NAME, &field_state.yellow_team[2]},
                                           {BLUE_ROBOT_0_NAME, &field_state.blue_team[0]},
                                           {BLUE_ROBOT_1_NAME, &field_state.blue_team[1]},
                                           {BLUE_ROBOT_2_NAME, &field_state.blue_team[2]},
                                           {BALL_NAME, &field_state.ball}});

    uint32_t size = model_states->name.size();

    for (uint32_t i = 0; i < size; i++) {
        EntityState* entity_state_ptr = lookup_table[model_states->name[i]];

        if (entity_state_ptr != nullptr) {
            gazebo_msgs::ModelState model_state;

            model_state.pose = model_states->pose[i];
            model_state.twist = model_states->twist[i];
            model_state.model_name = model_states->name[i];
            model_state.reference_frame = DEFAULT_REFERENCE_FRAME;

            (*entity_state_ptr) = ModelState_to_EntityState(&model_state);
        }
    }

    return field_state;
}
}  // namespace converter
}  // namespace travesim
