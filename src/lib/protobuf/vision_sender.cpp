/**
 * @file vision_sender.cpp
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 * @brief Vision data sender with UDP and protobuf
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
    fira_message::sim_to_ref::Environment environment_data = this->field_state_to_env_pb_msg(p_field_state);

    std::string buffer;
    environment_data.SerializeToString(&buffer);

    if (this->multicast_sender->send(buffer.c_str(), buffer.length()) == 0) {
        // warning
    }
}

fira_message::sim_to_ref::Environment VisionSender::field_state_to_env_pb_msg(FieldState* p_field_state) {
    fira_message::sim_to_ref::Environment env_data;

    // convert data

    return env_data;
}
}  // namespace proto
}  // namespace travesim
