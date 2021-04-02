#include <iostream>
#include <string>
#include <stdint.h>
#include <exception>
#include <boost/asio.hpp>
#include "boost/bind.hpp"

#include "travesim_adapters/multicast_receiver.hh"

#define DATA_MAX_LENGTH 1024

MulticastReceiver::MulticastReceiver(std::string multicast_address,
                                     short multicast_port, std::string listener_address) {
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

void MulticastReceiver::create_socket(const boost::asio::ip::address multicast_ip) {
    this->socket = new boost::asio::ip::udp::socket(io_context);

    // Create the socket so that multiple may be bound to the same address.
    this->socket->open(this->listener_endpoint.protocol());
    this->socket->set_option(boost::asio::ip::udp::socket::reuse_address(true));
    this->socket->bind(this->listener_endpoint);

    // Usar non blocking para leitura síncrona
    this->socket->non_blocking(true);

    // Join the multicast group.
    this->socket->set_option(boost::asio::ip::multicast::join_group(multicast_ip));
};

size_t MulticastReceiver::receive(std::array<char, 1024>* buffer) {
    size_t data_size;
    boost::system::error_code ec;

    data_size = this->socket->receive_from(boost::asio::buffer(*buffer), this->sender_endpoint, 0, ec);

    switch(ec.value()) {
        case boost::system::errc::success: {
            // Message received, do some log with self->sender_endpoint to register whoe is sending messages
            break;
        }

        case boost::asio::error::would_block: {
            data_size = 0;
            break;
        }

        default: {
            std::string error_msg = "Error in multicast receiver. Boost error: ";
            error_msg += ec.category().name();
            error_msg += " ";
            error_msg += std::to_string(ec.value());
            throw std::runtime_error(error_msg);
        }
    }

    return data_size;
};
