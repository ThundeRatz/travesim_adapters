/**
 * @file team_receiver.cpp
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

#include "travesim_adapters/protobuf/team_receiver.hpp"

/*****************************************
 * Private Constants
 *****************************************/

#define BUFFER_SIZE 1024U

/*****************************************
 * Public Methods Bodies Definitions
 *****************************************/

namespace travesim {
namespace proto {
TeamReceiver::TeamReceiver(const std::string receiver_address, const short receiver_port, bool is_yellow) {
    this->unicast_receiver =
        new udp::UnicastReceiver(receiver_address, receiver_port);
    this->unicast_receiver->force_specific_source(true);

    this->is_yellow = is_yellow;
}

TeamReceiver::~TeamReceiver() {
    delete this->unicast_receiver;
}

bool TeamReceiver::receive(TeamCommand* p_team_cmd) {
    char buffer[BUFFER_SIZE];

    if (this->unicast_receiver->receive(buffer, BUFFER_SIZE) > 0) {
        fira_message::sim_to_ref::Packet packet_data;
        packet_data.ParseFromArray(buffer, BUFFER_SIZE);

        if (packet_data.has_cmd()) {
            *p_team_cmd = this->packet_pb_msg_to_team_command(&packet_data);

            last_team_cmd = *p_team_cmd;

            return true;
        } else {
            *p_team_cmd = last_team_cmd;
        }
    } else {
        *p_team_cmd = last_team_cmd;
    }

    return false;
}

void TeamReceiver::set_receiver_endpoint(const std::string receiver_address, const short receiver_port) {
    this->unicast_receiver->set_receiver_endpoint(receiver_address,
                                                  receiver_port);
    this->unicast_receiver->reset();
}

void TeamReceiver::reset(void) {
    this->unicast_receiver->reset();
}

TeamCommand TeamReceiver::packet_pb_msg_to_team_command(fira_message::sim_to_ref::Packet* p_packet) {
    TeamCommand team_cmd;

    for (const auto& robot_cmd : p_packet->cmd().robot_commands()) {
        if (robot_cmd.yellowteam() == this->is_yellow) {
            int robot_id = robot_cmd.id();

            if ((robot_id < 0) || (robot_id >= NUM_OF_ROBOTS_PER_TEAM)) {
                ROS_WARN_STREAM("Error: Invalid robot id in team receiver!");
                continue;
            }

            if (isnanf(robot_cmd.wheel_left()) || isnanf(robot_cmd.wheel_right())) {
                ROS_WARN_STREAM("Error: Invalid robot speed in team receiver!");
                continue;
            }

            team_cmd.robot_command[robot_id].left_speed = robot_cmd.wheel_left();
            team_cmd.robot_command[robot_id].right_speed = robot_cmd.wheel_right();
        } else {
            ROS_WARN("Error: Team %s receiver and command colors don't match!",
                     this->is_yellow ? "yellow" : "blue");
        }
    }

    return team_cmd;
}
}  // namespace proto
}  // namespace travesim
