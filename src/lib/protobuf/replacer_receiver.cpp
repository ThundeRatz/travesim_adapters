/**
 * @file replacer_receiver.cpp
 *
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 *
 * @brief Team control data receiver with UDP and protobuf
 *
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 */

#include <ros/console.h>

#include "travesim_adapters/protobuf/replacer_receiver.hpp"

/*****************************************
 * Private Constants
 *****************************************/

#define BUFFER_SIZE 1024U

/*****************************************
 * Public Methods Bodies Definitions
 *****************************************/

namespace travesim {
namespace proto {
ReplacerReceiver::ReplacerReceiver(const std::string receiver_address, const short receiver_port,
                                   bool force_specific_source) {
    this->unicast_receiver = new udp::UnicastReceiver(receiver_address, receiver_port);
    this->unicast_receiver->force_specific_source(force_specific_source);
}

ReplacerReceiver::~ReplacerReceiver() {
    delete this->unicast_receiver;
}

bool ReplacerReceiver::receive(std::queue<EntityState>* p_replament_queue) {
    char buffer[BUFFER_SIZE];

    if (this->unicast_receiver->receive(buffer, BUFFER_SIZE) > 0) {
        fira_message::sim_to_ref::Packet packet_data;
        packet_data.ParseFromArray(buffer, BUFFER_SIZE);

        if (packet_data.has_replace()) {
            for (const auto& robot_replacement : packet_data.replace().robots()) {
                RobotState robot_state = this->robot_rplcmt_pb_to_robot_state(&robot_replacement);
                p_replament_queue->push(robot_state);
            }

            if (packet_data.replace().has_ball()) {
                const fira_message::sim_to_ref::BallReplacement ball = packet_data.replace().ball();
                EntityState ball_state = this->ball_rplcmt_pb_to_entity_state(&ball);
                p_replament_queue->push(ball_state);
            }

            return true;
        }
    }

    return false;
}

void ReplacerReceiver::set_receiver_endpoint(const std::string receiver_address, const short receiver_port) {
    this->unicast_receiver->set_receiver_endpoint(receiver_address, receiver_port);
    this->unicast_receiver->reset();
}

void ReplacerReceiver::reset(void) {
    this->unicast_receiver->reset();
}

EntityState ReplacerReceiver::ball_rplcmt_pb_to_entity_state(const fira_message::sim_to_ref::BallReplacement* p_ball_pb_msg) {
    EntityState ball_state;

    return ball_state;
}

RobotState ReplacerReceiver::robot_rplcmt_pb_to_robot_state(const fira_message::sim_to_ref::RobotReplacement* p_robot_pb_msg) {
    RobotState robot_state;

    return robot_state;
}

}  // namespace proto
}  // namespace travesim
