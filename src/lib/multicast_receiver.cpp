#include <iostream>
#include <string>
#include <stdint.h>
#include <boost/asio.hpp>
#include "boost/bind.hpp"

#include "travesim_adapters/multicast_receiver.hh"

#define DATA_MAX_LENGTH 1024

MulticastReceiver::MulticastReceiver(boost::asio::io_context& io_context, std::string multicast_address,
                                     short multicast_port, std::string listener_address) : socket(io_context) {
    const boost::asio::ip::address multicast_ip = boost::asio::ip::address::from_string(multicast_address);
    const boost::asio::ip::address listener_ip = boost::asio::ip::address::from_string(listener_address);

    // this->socket = boost::asio::ip::udp::socket(io_context);
    this->listener_endpoint = boost::asio::ip::udp::endpoint(listener_ip, multicast_port);

    this->configure_socket(multicast_ip);
};

MulticastReceiver::MulticastReceiver(boost::asio::io_context& io_context, std::string multicast_address,
                                     short multicast_port) : socket(io_context) {
    const boost::asio::ip::address multicast_ip = boost::asio::ip::address::from_string(multicast_address);

    // this->socket = boost::asio::ip::udp::socket(io_context);
    this->listener_endpoint = boost::asio::ip::udp::endpoint(multicast_ip, multicast_port);

    this->configure_socket(multicast_ip);
};

void MulticastReceiver::configure_socket(const boost::asio::ip::address multicast_ip) {
    // Create the socket so that multiple may be bound to the same address.
    this->socket.open(this->listener_endpoint.protocol());
    this->socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    this->socket.bind(this->listener_endpoint);

    // Usar non blocking para leitura síncrona
    this->socket.non_blocking(true);

    // Join the multicast group.
    this->socket.set_option(boost::asio::ip::multicast::join_group(multicast_ip));
};

size_t MulticastReceiver::receive(std::array<char, 1024>* buffer) {
    size_t data_size = 0;

    try {
        data_size = this->socket.receive_from(boost::asio::buffer(*buffer), this->sender_endpoint);

    } catch (boost::wrapexcept<boost::system::system_error>& e) {}

    return data_size;
};
