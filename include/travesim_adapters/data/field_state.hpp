/**
 * @file field_command.hpp
 *
 * @brief Field state data structure
 *
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 *
 * @date 04/2021
 */

#ifndef __FIELD_STATE_H__
#define __FIELD_STATE_H__

#include <iostream>
#include "travesim_adapters/data/data_constants.hpp"
#include "travesim_adapters/data/entity_state.hpp"

namespace travesim {
/**
 * @brief Data structure to hold the field state
 *
 */
class FieldState {
    public:
        /**
         * @brief Construct a new Field State object
         *
         */
        FieldState() = default;

        /**
         * @brief Output stream operator overloading
         *
         */
        friend std::ostream& operator <<(std::ostream& output, const FieldState& field_state);

        /**
         * @brief Field entities
         *
         */
        EntityState ball;
        EntityState yellow_team[NUM_OF_ROBOTS_PER_TEAM];
        EntityState blue_team[NUM_OF_ROBOTS_PER_TEAM];

        /**
         * @brief ODE time step
         *
         */
        unsigned int time_step;
};
}

#endif // __FIELD_STATE_H__
