/**
 * @file vision_sender.cpp
 *
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 *
 * @brief Vision data sender with UDP and protobuf
 *
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include "travesim_adapters/protobuf/vision_sender.hpp"

/*****************************************
 * Class Methods Bodies Definitions
 *****************************************/

namespace travesim {
namespace proto {
VisionSender::VisionSender(const std::string multicast_address, const short multicast_port) {
    this->multicast_sender = new udp::MulticastSender(multicast_address, multicast_port);
}

VisionSender::~VisionSender() {
    delete this->multicast_sender;
}

void VisionSender::send(FieldState* p_field_state) {
    fira_message::sim_to_ref::Environment env_data = this->field_state_to_env_pb_msg(p_field_state);

    std::string buffer;
    env_data.SerializeToString(&buffer);

    if (this->multicast_sender->send(buffer.c_str(), buffer.length()) == 0) {
        /**
         * @todo Warn send error
         */
    }
}

fira_message::sim_to_ref::Environment VisionSender::field_state_to_env_pb_msg(FieldState* p_field_state) {
    /**
     * @todo Update step and add field dimmensions
     */
    fira_message::sim_to_ref::Environment env_data;
    fira_message::Frame* frame = env_data.mutable_frame();

    // Set step
    env_data.set_step(p_field_state->time_step);

    // Set ball data
    fira_message::Ball* frame_ball = frame->mutable_ball();

    frame_ball->set_x(p_field_state->ball.position.x);
    frame_ball->set_y(p_field_state->ball.position.y);
    frame_ball->set_vx(p_field_state->ball.velocity.x);
    frame_ball->set_vy(p_field_state->ball.velocity.y);

    // Set yellow team
    for (int i = 0; i < NUM_OF_ROBOTS_PER_TEAM; i++) {
        fira_message::Robot* robot = frame->add_robots_yellow();

        robot->set_robot_id(i);

        robot->set_x(p_field_state->yellow_team[i].position.x);
        robot->set_y(p_field_state->yellow_team[i].position.y);
        robot->set_orientation(p_field_state->yellow_team[i].angular_position);

        robot->set_vx(p_field_state->yellow_team[i].velocity.x);
        robot->set_vy(p_field_state->yellow_team[i].velocity.y);
        robot->set_vorientation(p_field_state->yellow_team[i].angular_velocity);
    }

    // Set blue team
    for (int i = 0; i < NUM_OF_ROBOTS_PER_TEAM; i++) {
        fira_message::Robot* robot = frame->add_robots_blue();

        robot->set_robot_id(i);

        robot->set_x(p_field_state->blue_team[i].position.x);
        robot->set_y(p_field_state->blue_team[i].position.y);
        robot->set_orientation(p_field_state->blue_team[i].angular_position);

        robot->set_vx(p_field_state->blue_team[i].velocity.x);
        robot->set_vy(p_field_state->blue_team[i].velocity.y);
        robot->set_vorientation(p_field_state->blue_team[i].angular_velocity);
    }

    // Set field dimmensions
    fira_message::Field* field = env_data.mutable_field();

    field->set_width(FIELD_WIDTH_M);
    field->set_length(FIELD_LENGTH_M);
    field->set_goal_width(GOAL_WIDTH_M);
    field->set_goal_depth(GOAL_DEPTH_M);

    return env_data;
}
}  // namespace proto
}  // namespace travesim
