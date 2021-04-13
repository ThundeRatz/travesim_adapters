/**
 * @file multicast_sender.cpp
 * @author Lucas Haug <lucas.haug@thuneratz.org>
 * @author Lucas Schneider <lucas.schneider@thuneratz.org>
 * @brief Send data using UDP in multicast mode
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include <iostream>
#include <string>
#include <exception>
#include <boost/asio.hpp>

#include "travesim_adapters/udp/multicast_sender.hpp"

/*****************************************
 * Private Constants
 *****************************************/

#define NO_FLAGS 0U

/*****************************************
 * Class Methods Bodies Definitions
 *****************************************/

namespace travesim {
namespace udp {

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

    switch (ec.value()) {
        case boost::system::errc::success: {
            // Message sent. Log?
            break;
        }

        default: {
            throw boost::system::system_error(ec);
        }
    }

    return bytes_sent;
};
}  // namespace udp
}  // namespace travesim
