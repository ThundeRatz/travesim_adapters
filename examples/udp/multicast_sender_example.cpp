/**
 * @file multicast_sender_example.cpp
 * @author Lucas Haug <lucas.haug@thuneratz.org>
 * @author Lucas Schneider <lucas.schneider@thuneratz.org>
 * @brief Example on how to use the MulticastReceiver
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include <boost/asio.hpp>
#include <iostream>
#include <string>

#include "travesim_adapters/udp/multicast_sender.hpp"

/*****************************************
 * Main Function
 *****************************************/

int main() {
    const short multicast_port = 30001;
    const std::string multicast_address_str = "239.255.0.1";
    const int max_message_count = 20;

    try {
        boost::asio::io_context io_context;
        boost::asio::steady_timer my_timer(io_context);
        travesim::udp::MulticastSender my_sender(multicast_address_str, multicast_port);

        for (int i = 0; i < max_message_count; i++) {
            std::string msg = "Menssagem " + std::to_string(i);
            size_t bytes_sent = my_sender.send(msg.c_str(), msg.length());

            std::cout << msg << "\nBytes sent: " << bytes_sent << "\n\n";

            my_timer.expires_after(std::chrono::milliseconds(200));
            my_timer.wait();
        }

    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}