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
#include "travesim_adapters/data/field_state.hpp"

namespace travesim_adapters {
    std::ostream& operator<<(std::ostream& output, const FieldState& field_state) {
        output << "BALL STATE: " << std::endl;
        output << field_state.ball << std::endl;

        output << "TEAM YELLOW STATE: " << std::endl;
        for (int i = 0; i < NUM_OF_ROBOTS_PER_TEAM; i++) {
            output << "ROBOT " << i << ":" << std::endl;
            output << field_state.yellow_team[i] << std::endl;
        }

        output << std::endl;

        output << "TEAM BLUE STATE: " << std::endl;
        for (int i = 0; i < NUM_OF_ROBOTS_PER_TEAM; i++) {
            output << "ROBOT " << i << ":" << std::endl;
            output << field_state.blue_team[i] << std::endl;
        }

        output << std::endl;

        return output;
    }
}
