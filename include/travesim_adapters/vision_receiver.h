/**
 * @file vision_receiver.h
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#ifndef __VISION_RECEIVER_H__
#define __VISION_RECEIVER_H__

#include "ros/console.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"

#include <iostream>
#include <unordered_map>

#define YELLOW_ROBOT_0_NAME "yellow_team/robot_0"
#define YELLOW_ROBOT_1_NAME "yellow_team/robot_1"
#define YELLOW_ROBOT_2_NAME "yellow_team/robot_2"

#define BLUE_ROBOT_0_NAME "blue_team/robot_0"
#define BLUE_ROBOT_1_NAME "blue_team/robot_1"
#define BLUE_ROBOT_2_NAME "blue_team/robot_2"

#define BALL_NAME "vss_ball"

typedef std::unordered_map<std::string, int32_t> lookup_table_t;

class VisionReceiver {
  private:
    lookup_table_t lookup_table;
    const std::string topics[7] = { YELLOW_ROBOT_0_NAME,
                                    YELLOW_ROBOT_1_NAME,
                                    YELLOW_ROBOT_2_NAME,
                                    BLUE_ROBOT_0_NAME,
                                    BLUE_ROBOT_1_NAME,
                                    BLUE_ROBOT_2_NAME,
                                    BALL_NAME };

  public:
    VisionReceiver();
    // ~VisionReceiver();

    gazebo_msgs::ModelState world_state[7];

    void receive(const gazebo_msgs::ModelStates::ConstPtr& msg);
    int32_t model_name_to_index(std::string topic);
};

#endif  // __VISION_RECEIVER_H__
