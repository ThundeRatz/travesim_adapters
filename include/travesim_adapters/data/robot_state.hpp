/**
 * @file robot_state.hpp
 *
 * @brief Robot state data structure
 *
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 *
 * @date 04/2021
 */

#ifndef __ROBOT_STATE_H__
#define __ROBOT_STATE_H__

#include <iostream>
#include <gazebo_msgs/ModelState.h>

#include "travesim_adapters/data/entity_state.hpp"

namespace travesim {
/**
 * @brief Data structure to hold the state of a robot in the simulation
 *
 */
class RobotState :
    public EntityState {
    public:
        /**
         * @brief Construct a new Robot State object
         *
         */
        RobotState();

        /**
         * @brief Construct a new Robot State object
         *
         * @param position Position in meters
         * @param angular_position Angular position in radinans
         * @param velocity Velocity in m/s
         * @param angular_velocity Angular velocity in rad/s
         * @param is_yellow Wheter the robot is from the yellow team or not
         * @param id Identification number
         */
        RobotState(Vector2D position, double angular_position, Vector2D velocity, double angular_velocity,
                   bool is_yellow, int id);

        RobotState(EntityState* entity_state, bool is_yellow, int id);

        /**
         * @brief Construct a new Robot State object from Model State object
         *
         * @param model_state
         */
        RobotState(gazebo_msgs::ModelState* model_state);

        RobotState(gazebo_msgs::ModelState* model_state, bool is_yellow, int id);

        /**
         * @brief Convert object to data type used in Gazebo
         *
         * @return gazebo_msgs::ModelState converted data
         */
        gazebo_msgs::ModelState to_ModelState();

        /**
         * @brief Output stream operator overloading
         *
         */
        friend std::ostream& operator <<(std::ostream& output, const RobotState& robot_state);

        /**
         * @brief Public attributes
         *
         */
        bool is_yellow;
        int id;
};
}

#endif // __ROBOT_STATE_H__
