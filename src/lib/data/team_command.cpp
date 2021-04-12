/**
 * @file team_command.cpp
 *
 * @brief Team command data structure
 *
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 *
 * @date 04/2021
 */

#include <iomanip>
#include "travesim_adapters/data/team_command.hpp"

namespace travesim_adapters {
/*****************************************
 * RobotCommand Related
 *****************************************/

RobotCommand::RobotCommand(double left_speed, double right_speed) {
    this->left_speed = left_speed;
    this->right_speed = right_speed;
}

std::ostream& operator <<(std::ostream& output, const RobotCommand& command) {
    output << std::fixed << std::setprecision(PRINTING_DECIMAL_PRECISION);

    output << "LEFT SPEED: " << std::setw(PRINTING_MIN_WIDTH) << command.left_speed << std::endl;
    output << "RIGHT SPEED: " << std::setw(PRINTING_MIN_WIDTH) << command.right_speed;

    return output;
}

std::ostream& operator <<(std::ostream& output, const TeamCommand& command) {
    output << std::fixed << std::setprecision(PRINTING_DECIMAL_PRECISION);

    for (int i = 0; i < NUM_OF_ROBOTS_PER_TEAM; i++) {
        output << "ROBOT " << i << ":" << std::endl;
        output << command.robot_command[i] << std::endl;
        output << std::endl;
    }

    return output;
}
}
