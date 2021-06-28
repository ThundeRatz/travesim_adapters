/**
 * @file replacer_adapter.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief Teams commands adapter execution file
 * @date 06/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include <queue>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelStates.h>

#include "travesim_adapters/ros/replacer_sender.hpp"
#include "travesim_adapters/protobuf/replacer_receiver.hpp"
#include "travesim_adapters/data/converter/ros_side.hpp"


int main(int argc, char** argv) {
    ros::init(argc, argv, "teams_adapter");
    ros::NodeHandle nh;

    int32_t send_rate;
    travesim::ros_side::ReplacerSender replacer_sender;
    travesim::ros_side::state_vector_t model_state_vector;

    const std::string receiver_address = "127.0.0.1";
    const short receiver_port = 20011;

    std::queue<std::shared_ptr<travesim::EntityState>> states_queue;

    travesim::proto::ReplacerReceiver replacer_receiver(receiver_address, receiver_port, true);

    nh.param<int32_t>("send_rate", send_rate, 120);

    ros::Rate loop_rate(send_rate);

    ROS_INFO_STREAM("Replacer adapter started with loop rate " << send_rate);

    while (ros::ok()) {

        bool received_new_msg = replacer_receiver.receive(&states_queue);

        if (received_new_msg) {
            replacer_sender.send_command(travesim::ros_side::PAUSE);

            while (!states_queue.empty()) {
                std::shared_ptr<travesim::RobotState> state = std::dynamic_pointer_cast<travesim::RobotState>(
                    states_queue.front());

                gazebo_msgs::ModelStatePtr gazebo_state_msg;
                if (state != nullptr) {
                    *gazebo_state_msg = travesim::converter::RobotState_to_ModelState(state.get());
                } else {
                    *gazebo_state_msg = travesim::converter::EntityState_to_ModelState(state.get());
                }
                model_state_vector.push_back(gazebo_state_msg);

                states_queue.pop();
            }

            replacer_sender.set_models_state(&model_state_vector);

            replacer_sender.send_command(travesim::ros_side::RESUME);

            model_state_vector.clear();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
