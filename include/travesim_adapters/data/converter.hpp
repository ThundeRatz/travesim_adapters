/**
 * @file converter.hpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#ifndef __CONVERTER_H__
#define __CONVERTER_H__

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

#include "travesim_adapters/data/data_constants.hpp"
#include "travesim_adapters/data/robot_state.hpp"
#include "travesim_adapters/data/entity_state.hpp"

namespace travesim {
namespace converter {
/**
 * @brief Function to convert geometry_msgs::Point to travesim::Vector2D
 *
 * @param point Data to be converted
 * @return travesim::Vector2D Converted data
 */
travesim::Vector2D Point_to_Vector2D(geometry_msgs::Point* point);

/**
 * @brief Function to convert geometry_msgs::Vector3 to travesim::Vector2D
 *
 * @param vector3 Data to be converted
 * @return travesim::Vector2D Converted data
 */
travesim::Vector2D Vector3_to_Vector2D(geometry_msgs::Vector3* vector3);

/**
 * @brief Function to convert gazebo_msgs::ModelState to travesim::EntityState
 *
 * @param model_state Data to be converted
 * @return travesim::EntityState Converted data
 */
travesim::EntityState ModelState_to_EntityState(gazebo_msgs::ModelState* model_state);

/**
 * @brief Function to convert gazebo_msgs::ModelState to travesim::RobotState
 *
 * @param model_state Data to be converted
 * @param is_yellow True if the robot is from the yellow team, false otherwise
 * @param id Id number of the robot
 * @return travesim::RobotState Converte data
 */
travesim::RobotState ModelState_to_RobotState(gazebo_msgs::ModelState* model_state, bool is_yellow = true, int id = 0);

/**
 * @brief Function to convert travesim::Vector2D to geometry_msgs::Point
 *
 * @param vector2d Data to be converted
 * @return geometry_msgs::Point Converted data
 */
geometry_msgs::Point Vector2D_to_Point(Vector2D* vector2d);

/**
 * @brief Function to convert travesim::Vector2D to geometry_msgs::Vector3
 *
 * @param vector2d Data to be converted
 * @return geometry_msgs::Vector3 Converted data
 */
geometry_msgs::Vector3 Vector2D_to_Vector3(Vector2D* vector2d);

/**
 * @brief Function to convert travesim::EntityState to gazebo_msgs::ModelState
 *
 * @param entity_state Data to be converted
 * @return gazebo_msgs::ModelState Converted data
 */
gazebo_msgs::ModelState EntityState_to_ModelState(EntityState* entity_state);

/**
 * @brief Function to convert travesim::RobotState to gazebo_msgs::ModelState
 *
 * @param entity_state Data to be converted
 * @return gazebo_msgs::ModelState Converted data
 */
gazebo_msgs::ModelState RobotState_to_ModelState(RobotState* robot_state);
}  // namespace converter
}  // namespace travesim

#endif // __CONVERTER_H__
