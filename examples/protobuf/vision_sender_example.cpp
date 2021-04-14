/**
 * @file multicast_sender_example.cpp
 * @author Lucas Haug <lucas.haug@thuneratz.org>
 * @brief Example on how to use the MulticastReceiver
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include <iostream>
#include <string>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/text_format.h>

#include "travesim_adapters/protobuf/vision_sender.hpp"

/*****************************************
 * Main Function
 *****************************************/

int main() {
    const short multicast_port = 30001;
    const std::string multicast_address_str = "239.255.0.1";

    travesim::proto::VisionSender my_sender(multicast_address_str, multicast_port);

    travesim::FieldState field_state;

    field_state.ball.position.x = 2.4;

    fira_message::sim_to_ref::Environment env_data = my_sender.field_state_to_env_pb_msg(&field_state);

    std::string buffer;

    google::protobuf::util::JsonPrintOptions options;
    options.add_whitespace = true;
    options.always_print_primitive_fields = true;

    google::protobuf::util::MessageToJsonString(env_data, &buffer, options);

    std::cout << buffer;

    return 0;
}
