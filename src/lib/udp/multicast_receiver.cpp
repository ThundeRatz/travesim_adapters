/**
 * @file multicast_receiver.cpp
 *
 * @author Lucas Haug <lucas.haug@thuneratz.org>
 * @author Lucas Schneider <lucas.schneider@thuneratz.org>
 *
 * @brief Receiver data using UDP in multicast mode
 *
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
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
    const boost::asio::ip::address multicast_boost_addr = boost::asio::ip::address::from_string(multicast_address);
    const boost::asio::ip::address listener_boost_addr = boost::asio::ip::address::from_string(listener_address);

    this->listener_endpoint = boost::asio::ip::udp::endpoint(listener_boost_addr, multicast_port);
    this->multicast_address = multicast_boost_addr;

    this->create_socket();

    this->specific_source = false;
};

MulticastReceiver::MulticastReceiver(std::string multicast_address, short multicast_port) :
    MulticastReceiver(multicast_address, multicast_port, multicast_address) {
};

MulticastReceiver::~MulticastReceiver() {
    this->close_socket();
    delete this->socket;
};

void MulticastReceiver::create_socket() {
    this->socket = new boost::asio::ip::udp::socket(io_context);

    // Create the socket so that multiple may be bound to the same address.
    this->socket->open(this->listener_endpoint.protocol());
    this->socket->set_option(boost::asio::ip::udp::socket::reuse_address(true));
    this->socket->bind(this->listener_endpoint);

    // Use non blocking for syncronous reading
    this->socket->non_blocking(true);

    // Join the multicast group.
    this->socket->set_option(boost::asio::ip::multicast::join_group(this->multicast_address));
};

void MulticastReceiver::close_socket() {
    this->socket->close();
};

size_t MulticastReceiver::receive(char* buffer, const size_t buffer_size) {
    size_t bytes_received;
    boost::system::error_code ec;
    boost::asio::ip::udp::endpoint current_endpoint;

    bytes_received =
        this->socket->receive_from(boost::asio::buffer(buffer, buffer_size), current_endpoint, NO_FLAGS, ec);

    if (this->specific_source) {
        if (this->sender_endpoint == boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)) {
            this->sender_endpoint = current_endpoint;
        } else if (this->sender_endpoint != current_endpoint) {
            if (current_endpoint != boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)) {
                std::string error_msg = "Error in multicast receiver. Any-source multicast not enabled.";
                throw std::runtime_error(error_msg);
            }
        }
    }

    switch (ec.value()) {
        case boost::system::errc::success: {
            /**
             * @todo sender_endpoint loggin?
             */
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

void MulticastReceiver::force_specific_source(bool specific_source) {
    this->specific_source = specific_source;
};
}  // namespace udp
}  // namespace travesim
