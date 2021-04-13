/**
 * @file entity_state.hpp
 *
 * @brief Entity state data structure
 *
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 *
 * @date 04/2021
 */

#ifndef __ENTITY_STATE_H__
#define __ENTITY_STATE_H__

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelState.h>

#include <iostream>

namespace travesim {
/**
 * @brief Data structure to hold a two dimensional vector
 *
 */
class Vector2D {
    public:
        /**
         * @brief Construct a new Vector2D object
         *
         * @param x x quota
         * @param y y quota
         */
        Vector2D(double x = 0, double y = 0);

        /**
         * @brief Construct a new Vector 2D object from Vector3 object
         *
         * @param vector3
         */
        Vector2D(geometry_msgs::Vector3* vector3);

        /**
         * @brief Construct a new Vector 2D object from Point object
         *
         * @param point
         */
        Vector2D(geometry_msgs::Point* point);

        /**
         * @brief Convert object to data type used in Gazebo
         *
         * @return geometry_msgs::Vector3
         */
        geometry_msgs::Vector3 to_Vector3();

        /**
         * @brief Convert object to data type used in Gazebo
         *
         * @return geometry_msgs::Vector3
         */
        geometry_msgs::Point to_Point();

        /**
         * @brief Perform a counter-clockwise rotation of the vector
         *
         * @param theta Rotation angle in radians
         */
        void rotate(double theta);

        /**
         * @brief Output stream operator overloading
         *
         */
        friend std::ostream& operator <<(std::ostream& output, const Vector2D& vector_2d);

        /**
         * @brief Public attributes
         *
         */
        double x;
        double y;
};

/**
 * @brief Data structure to hold the state of a entity in the simulation
 *
 */
class EntityState {
    public:
        /**
         * @brief Construct a new Entity State object
         *
         */
        EntityState();

        /**
         * @brief Construct a new Entity State object
         *
         * @param position Position in meters
         * @param angular_position Angular position in radinans
         * @param velocity Velocity in m/s
         * @param angular_velocity Angular velocity in rad/s
         */
        EntityState(Vector2D position, double angular_position, Vector2D velocity, double angular_velocity);

        /**
         * @brief Construct a new Entity State object from Model State object
         *
         * @param model_state
         */
        EntityState(gazebo_msgs::ModelState* model_state);

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
        friend std::ostream& operator <<(std::ostream& output, const EntityState& entity_state);

        /**
         * @brief Position in meters
         *
         */
        Vector2D position;

        /**
         * @brief Velocity in m/s
         *
         */
        Vector2D velocity;

        /**
         * @brief Angular position in radinans
         *
         */
        double angular_position;

        /**
         * @brief Angular velocity in rad/s
         *
         */
        double angular_velocity;
};
}

#endif // __ENTITY_STATE_H__
