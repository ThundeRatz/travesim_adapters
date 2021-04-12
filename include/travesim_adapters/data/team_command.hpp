/**
 * @file team_command.hpp
 *
 * @brief Team command data structure
 *
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 *
 * @date 04/2021
 */

#ifndef __TEAM_COMMAND_H__
#define __TEAM_COMMAND_H__

#include <iostream>
#include "travesim_adapters/data/data_constants.hpp"

namespace travesim_adapters {
/**
 * @brief Data structure to hold the command for a robot
 *
 */
class RobotCommand {
    public:
        /**
         * @brief Construct a new Robot Command object
         *
         * @param left_speed Command left speed
         * @param right_speed Command right speed
         */
        RobotCommand(double left_speed = 0, double right_speed = 0);

        /**
         * @brief Output stream operator overloading
         *
         */
        friend std::ostream& operator <<(std::ostream& output, const RobotCommand& command);

        /**
         * @brief Public attributes
         *
         */
        double left_speed;
        double right_speed;
};

/**
 * @brief Data structure to hold the command for a team
 *
 */
class TeamCommand {
    public:
        /**
         * @brief Construct a new Team Command object
         *
         */
        TeamCommand() = default;

        /**
         * @brief Output stream operator overloading
         *
         */
        friend std::ostream& operator <<(std::ostream& output, const TeamCommand& command);

        /**
         * @brief Public attributes
         *
         */
        RobotCommand robot_command[NUM_OF_ROBOTS_PER_TEAM];
};
}

#endif // __TEAM_COMMAND_H__
