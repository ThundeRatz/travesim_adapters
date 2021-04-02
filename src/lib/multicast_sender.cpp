#include <iostream>
#include <string>
#include <exception>
#include <boost/asio.hpp>

#include "travesim_adapters/multicast_sender.hh"

#define NO_FLAGS 0U

MulticastSender::MulticastSender(std::string multicast_address, short multicast_port) {
    const boost::asio::ip::address multicast_ip = boost::asio::ip::address::from_string(multicast_address);

    this->endpoint = boost::asio::ip::udp::endpoint(multicast_ip, multicast_port);

    this->socket = new boost::asio::ip::udp::socket(this->io_context);
    this->socket->open(this->endpoint.protocol());
};

MulticastSender::~MulticastSender() {
    this->socket->close();

    delete this->socket;
};

size_t MulticastSender::send(const char* buffer, const size_t buffer_size) {
    size_t bytes_sent;
    boost::system::error_code ec;

    bytes_sent = this->socket->send_to(boost::asio::buffer(buffer, buffer_size), this->endpoint, NO_FLAGS, ec);

    switch(ec.value()) {
        case boost::system::errc::success: {
            // Message sent. Log?
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

    return bytes_sent;
};
