/**
 * @file multicast_sender.hpp
 *
 * @author Lucas Haug <lucas.haug@thuneratz.org>
 * @author Lucas Schneider <lucas.schneider@thuneratz.org>

 * @brief Send data using UDP in multicast mode
 *
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 */

#include <boost/asio.hpp>
#include <string>

#ifndef __MULTICAST_SENDER_H__
#define __MULTICAST_SENDER_H__

namespace travesim {
namespace udp {
/**
 * @brief Sender class using UDP in multicast mode
 */
class MulticastSender {
    public:
        /**
         * @note The default construct cannot be used
         */
        MulticastSender() = delete;

        /**
         * @brief Construct a new Multicast Sender object
         *
         * @param multicast_address Multicas group address
         * @param multicast_port Multicas group port
         */
        MulticastSender(const std::string multicast_address, const short multicast_port);

        /**
         * @brief Destroy the Multicast Sender object
         */
        ~MulticastSender();

        /**
         * @brief Send data using UDP
         *
         * @param buffer Buffer to be sent
         * @param buffer_size Size of the buffer to be sent
         *
         * @return size_t Number of bytes sent
         */
        size_t send(const char* buffer, const size_t buffer_size);

    private:
        boost::asio::io_context io_context;      /**< boost/asio I/O execution context */
        boost::asio::ip::udp::socket* socket;    /**< Network socket */
        boost::asio::ip::udp::endpoint endpoint; /**< Multicast address and port pair */
};
}  // namespace udp
}  // namespace travesim

#endif // __MULTICAST_SENDER_H__
