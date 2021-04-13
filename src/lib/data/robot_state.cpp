/**
 * @file robot_state.cpp
 *
 * @brief Robot state data structure
 *
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 *
 * @date 04/2021
 */

#include <iomanip>
#include "travesim_adapters/data/robot_state.hpp"
#include "travesim_adapters/data/data_constants.hpp"

namespace travesim {
/*****************************************
 * RobotState Related
 *****************************************/

RobotState::RobotState() : RobotState(Vector2D(), 0, Vector2D(), 0, true, 0) {
}

RobotState::RobotState(Vector2D position, double angular_position, Vector2D velocity, double angular_velocity,
                       bool is_yellow, int id) :
    EntityState(position, angular_position, velocity, angular_velocity) {
    this->is_yellow = is_yellow;
    this->id = id;
}

RobotState::RobotState(gazebo_msgs::ModelState* model_state) : RobotState(model_state, true, 0) {
}

RobotState::RobotState(gazebo_msgs::ModelState* model_state, bool is_yellow, int id) :
    EntityState(model_state) {
    this->is_yellow = is_yellow;
    this->id = id;
}

gazebo_msgs::ModelState RobotState::to_ModelState() {
    gazebo_msgs::ModelState retval = EntityState::to_ModelState();

    std::string base_name = this->is_yellow ? "yellow_team/robot_" : "blue_team/robot_";
    retval.model_name = base_name.append(std::to_string(this->id));

    return retval;
}

std::ostream& operator <<(std::ostream& output, const RobotState& robot_state) {
    output << std::fixed << std::setprecision(PRINTING_DECIMAL_PRECISION);

    output << "TEAM YELLOW: " << robot_state.is_yellow << std::endl;
    output << "ROBOT ID: " << robot_state.id << std::endl;

    output << "POSITION: " << std::endl;
    output << robot_state.position << " | ";
    output << "THETA: "<< std::setw(PRINTING_MIN_WIDTH) << robot_state.angular_position << std::endl;

    output << "VELOCITY: " << std::endl;
    output << robot_state.velocity << " | ";
    output << "THETA: "<< std::setw(PRINTING_MIN_WIDTH) << robot_state.angular_velocity << std::endl;

    return output;
}
}
