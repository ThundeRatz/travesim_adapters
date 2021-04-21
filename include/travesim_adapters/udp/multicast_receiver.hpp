/**
 * @file multicast_receiver.hpp
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

#include <boost/asio.hpp>
#include <string>

#ifndef __MULTICAST_RECEIVER_H__
#define __MULTICAST_RECEIVER_H__

namespace travesim {
namespace udp {
/**
 * @brief Receiver class using UDP in multicast mode
 */
class MulticastReceiver {
    public:
        /**
         * @brief Construct a new Multicast Receiver object
         *
         * @param multicast_address Multicas group address
         * @param multicast_port Multicast group port
         * @param listener_address Listener address, has a filtering role, setting
         *                         where the data may be received
         */
        MulticastReceiver(const std::string multicast_address, const short multicast_port,
                          const std::string listener_address);

        /**
         * @brief Construct a new Multicast Receiver object
         *
         * @param multicast_address Multicas group address
         * @param multicast_port Multicast group port
         *
         * @note Use multicast address as listen address
         */
        MulticastReceiver(const std::string multicast_address, const short multicast_port);

        /**
         * @brief Destroy the Multicast Receiver object
         */
        ~MulticastReceiver();

        /**
         * @brief Receive data using UDP
         *
         * @param buffer Buffet to store data
         * @param buffer_size Size of the buffer where to store data
         *
         * @return size_t Number of bytes received
         */
        size_t receive(char* buffer, const size_t buffer_size);

        /**
         * @brief Set wheter to enable any source or source specific multicast.
         *        True for SSM, false for SSM, default is false.
         *
         * @param specific_source Whether to enable source specific or not.
         */
        void force_specific_source(bool specific_source);

    private:
        bool specific_source; /**< True for SSM, false for ASM, default is false */

        boost::asio::io_context io_context;   /**< boost/asio I/O execution context */
        boost::asio::ip::udp::socket* socket; /**< Network socket*/

        /**
         * @brief Endpoints: addresses and ports pairs
         */
        boost::asio::ip::udp::endpoint sender_endpoint;
        boost::asio::ip::udp::endpoint listener_endpoint;

        /**
         * @brief Create a socket object
         *
         * @param multicast_address Multicas group address
         */
        void create_socket(const boost::asio::ip::address multicast_address);
};
}  // namespace udp
}  // namespace travesim

#endif // __MULTICAST_RECEIVER_H__
