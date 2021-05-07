/**
 * @file replacer_receiver_example.cpp
 *
 * @author Lucas Haug <lucas.haug@thuneratz.org>
 * @author Lucas Schneider <lucas.schneider@thuneratz.org>
 *
 * @brief Example to read data published from VSSReplacer
 *
 * @date 05/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 */

#include <queue>
#include <iostream>

#include "travesim_adapters/protobuf/replacer_receiver.hpp"

/*****************************************
 * Main Function
 *****************************************/

int main(int argc, char* argv[]) {
    const std::string receiver_address = "127.0.0.1";
    const short receiver_port = 20011;

    std::queue<travesim::EntityState> states_queue;

    try {
        travesim::proto::ReplacerReceiver replacer_receiver(receiver_address, receiver_port, true);

        while (true) {
            bool received_new_msg = replacer_receiver.receive(&states_queue);

            if (received_new_msg) {
                while (!states_queue.empty()) {
                    travesim::EntityState* state = dynamic_cast<travesim::RobotState*>(&states_queue.front());

                    if (state != nullptr) {
                        std::cout << dynamic_cast<travesim::RobotState&>(states_queue.front());
                    } else {
                        std::cout << states_queue.front();
                    }

                    states_queue.pop();

                    std::cout << std::endl;

                    delete state;
                }
            }
        }
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}
