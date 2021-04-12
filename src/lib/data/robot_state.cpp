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

namespace travesim_adapters {
    /*****************************************
     * RobotState Related
     *****************************************/

    RobotState::RobotState() : RobotState(Vector2D(), 0, Vector2D(), 0, true, 0) {
    }

    RobotState::RobotState(Vector2D position, double angular_position, Vector2D velocity, double angular_velocity, bool team_yellow, int id) :
        EntityState(position, angular_position, velocity,  angular_velocity) {
        this->team_yellow = team_yellow;
        this->id = id;
    }

    std::ostream& operator<<(std::ostream& output, const RobotState& robot_state) {
        output << "TEAM YELLOW: " << robot_state.team_yellow << std::endl;
        output << "ROBOT ID: " << robot_state.id << std::endl;
        output << "POSITION: " << std::endl;
        output << robot_state.position << " | ";
        output << "THETA: "<< std::setw(5) << robot_state.angular_position << std::endl;
        output << "VELOCITY: " << std::endl;
        output << robot_state.velocity << " | ";
        output << "THETA: "<< std::setw(5) << robot_state.angular_velocity << std::endl;

        return output;
    }
}
