/**
 * @file vision_adapter.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include "ros/ros.h"
#include "ros/console.h"
#include "gazebo_msgs/ModelStates.h"
#include "travesim_adapters/vision_receiver.h"

#include <iostream>

int main(int argc, char** argv)
{
  int32_t send_rate;
  VisionReceiver vision_receiver;

  ros::init(argc, argv, "vision_adapter");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 2, &VisionReceiver::receive, &vision_receiver);

  nh.param<int32_t>("send_rate", send_rate, 60);

  ros::Rate loop_rate(send_rate);

  ROS_INFO_STREAM("Vision adapter started with loop rate " << send_rate);

  while (ros::ok())
  {
    // Send message with protobuf
    ROS_INFO_STREAM(vision_receiver.world_state[0].model_name
                    << " X: " << vision_receiver.world_state[0].pose.position.x
                    << " Y: " << vision_receiver.world_state[0].pose.position.x);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
