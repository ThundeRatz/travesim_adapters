/**
 * @file multicast_receiver.cpp
 * @author Lucas Haug <lucas.haug@thuneratz.org>
 * @author Lucas Schneider <lucas.schneider@thuneratz.org>
 * @brief Receiver data using UDP in multicast mode
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include <iostream>
#include <string>
#include <exception>
#include <boost/asio.hpp>
#include "boost/bind.hpp"

#include "travesim_adapters/udp/multicast_receiver.hpp"

/*****************************************
 * Private Constants
 *****************************************/

#define NO_FLAGS 0U

/*****************************************
 * Class Methods Bodies Definitions
 *****************************************/

namespace travesim {
namespace udp {
MulticastReceiver::MulticastReceiver(std::string multicast_address, short multicast_port,
                                     std::string listener_address) {
    const boost::asio::ip::address multicast_ip = boost::asio::ip::address::from_string(multicast_address);
    const boost::asio::ip::address listener_ip = boost::asio::ip::address::from_string(listener_address);

    this->listener_endpoint = boost::asio::ip::udp::endpoint(listener_ip, multicast_port);

    this->create_socket(multicast_ip);
};

MulticastReceiver::MulticastReceiver(std::string multicast_address, short multicast_port) {
    const boost::asio::ip::address multicast_ip = boost::asio::ip::address::from_string(multicast_address);

    this->listener_endpoint = boost::asio::ip::udp::endpoint(multicast_ip, multicast_port);

    this->create_socket(multicast_ip);
};

MulticastReceiver::~MulticastReceiver() {
    this->socket->close();
    delete this->socket;
}

void MulticastReceiver::create_socket(const boost::asio::ip::address multicast_address) {
    this->socket = new boost::asio::ip::udp::socket(io_context);

    // Create the socket so that multiple may be bound to the same address.
    this->socket->open(this->listener_endpoint.protocol());
    this->socket->set_option(boost::asio::ip::udp::socket::reuse_address(true));
    this->socket->bind(this->listener_endpoint);

    // Use non blocking for syncronous reading
    this->socket->non_blocking(true);

    // Join the multicast group.
    this->socket->set_option(boost::asio::ip::multicast::join_group(multicast_address));
};

size_t MulticastReceiver::receive(char* buffer, const size_t buffer_size) {
    size_t bytes_received;
    boost::system::error_code ec;
    boost::asio::ip::udp::endpoint current_endpoint;

    bytes_received =
        this->socket->receive_from(boost::asio::buffer(buffer, buffer_size), current_endpoint, NO_FLAGS, ec);

    if (this->sender_endpoint == boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)) {
        this->sender_endpoint = current_endpoint;
    } else if (this->sender_endpoint != current_endpoint) {
        if (current_endpoint != boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)) {
            std::string error_msg = "Error in multicast receiver. Any-source multicast not supported.";
            throw std::runtime_error(error_msg);
        }
    }

    switch (ec.value()) {
        case boost::system::errc::success: {
            // Message received, do some log with self->sender_endpoint to register whoe is sending
            break;
        }

        case boost::asio::error::would_block: {
            bytes_received = 0;
            break;
        }

        default: {
            throw boost::system::system_error(ec);
        }
    }

    return bytes_received;
};
}  // namespace udp
}  // namespace travesim
