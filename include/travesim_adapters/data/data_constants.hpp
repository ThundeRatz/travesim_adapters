/**
 * @file data_constants.hpp
 *
 * @brief General constants for the data structures
 *
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 *
 * @date 04/2021
 */

#ifndef __DATA_CONSTANTS_H__
#define __DATA_CONSTANTS_H__

/**
 * @brief Robots related constants
 */
#define NUM_OF_ROBOTS_PER_TEAM 3
#define NUM_OF_TOPICS_PER_TEAM (NUM_OF_ROBOTS_PER_TEAM * 2)
#define NUM_OF_ENTITIES_IN_FIELD (2 * NUM_OF_ROBOTS_PER_TEAM + 1)

/**
 * @brief Printing output configuration constants
 */
#define PRINTING_MIN_WIDTH 6U
#define PRINTING_DECIMAL_PRECISION 3U

/**
 * @brief Field and goal constants
 */
#define FIELD_WIDTH_M 1.3F
#define FIELD_LENGTH_M 1.5F
#define GOAL_WIDTH_M 0.4F
#define GOAL_DEPTH_M 0.1F

#endif // __DATA_CONSTANTS_H__
