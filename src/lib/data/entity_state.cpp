/**
 * @file entity_state.cpp
 *
 * @brief Entity state data structure
 *
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 *
 * @date 04/2021
 */

#include <iomanip>
#include "travesim_adapters/data/entity_state.hpp"

namespace travesim_adapters {
    Vector2D::Vector2D(double x, double y) {
        this->x = x;
        this->y = y;
    }

    std::ostream& operator<<(std::ostream& output, const Vector2D& vector_2d) {
        output << "X: " << std::setw(5) << vector_2d.x << " | ";
        output << "Y: " << std::setw(5) << vector_2d.y;

        return output;
    }

    EntityState::EntityState() : EntityState(Vector2D(), 0.0, Vector2D(), 0.0){
    }

    EntityState::EntityState(Vector2D position, double angular_position, Vector2D velocity, double angular_velocity) {
        this->position = position;
        this->angular_position = angular_position;
        this->velocity = velocity;
        this->angular_velocity = angular_velocity;
    }

    std::ostream& operator<<(std::ostream& output, const EntityState& entity_state) {
        output << "POSITION: " << std::endl;
        output << entity_state.position << " | ";
        output << "THETA: "<< std::setw(5) << entity_state.angular_position << std::endl;
        output << "VELOCITY: " << std::endl;
        output << entity_state.velocity << " | ";
        output << "THETA: "<< std::setw(6) << entity_state.angular_velocity << std::endl;

        return output;
    }
}
