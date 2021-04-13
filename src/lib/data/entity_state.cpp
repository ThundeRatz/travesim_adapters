/**
 * @file entity_state.cpp
 *
 * @brief Entity state data structure
 *
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 *
 * @date 04/2021
 */

#include <geometry_msgs/Point.h>
#include <cmath>
#include <iomanip>
#include "travesim_adapters/data/entity_state.hpp"
#include "travesim_adapters/data/data_constants.hpp"

#define DEFAULT_Z_VALUE 0.022
#define DEFAULT_ENTITY_NAME "vss_ball"
#define DEFAULT_REFERENCE_FRAME "world"

#define quaternion_to_theta(q0, q1, q2, q3) atan2(2*(q0*q1+q2*q3), 1 - 2*(q1*q1 + q2*q2))

namespace travesim {
/*****************************************
 * Vector2D Related
 *****************************************/

Vector2D::Vector2D(double x, double y) {
    this->x = x;
    this->y = y;
}

Vector2D::Vector2D(geometry_msgs::Vector3* vector3) {
    this->x = vector3->x;
    this->y = vector3->y;
}

Vector2D::Vector2D(geometry_msgs::Point* point) {
    this->x = point->x;
    this->y = point->y;
}

void Vector2D::rotate(double theta) {
    double old_x = this->x, old_y = this->y;

    this->x = cos(theta)*old_x + sin(theta)*old_y;
    this->y = -sin(theta)*old_x + cos(theta)*old_y;
}

geometry_msgs::Vector3 Vector2D::to_Vector3() {
    geometry_msgs::Vector3 retval;

    retval.x = this->x;
    retval.y = this->y;
    retval.z = 0;

    return retval;
}

geometry_msgs::Point Vector2D::to_Point() {
    geometry_msgs::Point retval;

    retval.x = this->x;
    retval.y = this->y;
    retval.z = DEFAULT_Z_VALUE;

    return retval;
}

std::ostream& operator <<(std::ostream& output, const Vector2D& vector_2d) {
    output << std::fixed << std::setprecision(PRINTING_DECIMAL_PRECISION);

    output << "X: " << std::setw(PRINTING_MIN_WIDTH) << vector_2d.x << " | ";
    output << "Y: " << std::setw(PRINTING_MIN_WIDTH) << vector_2d.y;

    return output;
}

/*****************************************
 * EntityState Related
 *****************************************/

EntityState::EntityState() : EntityState(Vector2D(), 0.0, Vector2D(), 0.0) {
}

EntityState::EntityState(Vector2D position, double angular_position, Vector2D velocity, double angular_velocity) {
    this->position = position;
    this->angular_position = angular_position;
    this->velocity = velocity;
    this->angular_velocity = angular_velocity;
}

EntityState::EntityState(gazebo_msgs::ModelState* model_state) {
    this->position = Vector2D(&model_state->pose.position);

    this->angular_position = quaternion_to_theta(model_state->pose.orientation.w, model_state->pose.orientation.x,
                                                 model_state->pose.orientation.y, model_state->pose.orientation.z);

    this->velocity = Vector2D(&model_state->twist.linear);

    this->angular_velocity = model_state->twist.angular.z;
}

gazebo_msgs::ModelState EntityState::to_ModelState() {
    gazebo_msgs::ModelState retval;

    retval.model_name = DEFAULT_ENTITY_NAME;
    retval.reference_frame = DEFAULT_REFERENCE_FRAME;

    retval.pose.position = this->position.to_Point();

    retval.pose.orientation.w = cos(this->angular_position/2);
    retval.pose.orientation.x = sin(this->angular_position/2);
    retval.pose.orientation.y = 0;
    retval.pose.orientation.z = 0;

    retval.twist.linear = this->velocity.to_Vector3();

    retval.twist.angular.x = 0;
    retval.twist.angular.y = 0;
    retval.twist.angular.z = this->angular_velocity;

    return retval;
}

std::ostream& operator <<(std::ostream& output, const EntityState& entity_state) {
    output << std::fixed << std::setprecision(PRINTING_DECIMAL_PRECISION);

    output << "POSITION: " << std::endl;
    output << entity_state.position << " | ";
    output << "THETA: "<< std::setw(PRINTING_MIN_WIDTH) << entity_state.angular_position << std::endl;

    output << "VELOCITY: " << std::endl;
    output << entity_state.velocity << " | ";
    output << "THETA: "<< std::setw(PRINTING_MIN_WIDTH) << entity_state.angular_velocity << std::endl;

    return output;
}
}
