/**
 * @file receiver.hpp
 *
 * @author Lucas Haug <lucas.haug@thuneratz.org>
 * @author Lucas Schneider <lucas.schneider@thuneratz.org>
 *
 * @brief Receiver data using UDP
 *
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 */

#include <boost/asio.hpp>
#include <string>

#ifndef __RECEIVER_H__
#define __RECEIVER_H__

namespace travesim {
namespace udp {
/**
 * @brief Receiver class using UDP
 */
class Receiver {
    public:
        /**
         * @brief Construct a new Receiver object
         *
         * @param receiver_address Address where to send data
         * @param receiver_port Port where to send data
         *
         * @note open_socket() must be called in the child's constructor
         */
        Receiver(const std::string receiver_address, const short receiver_port);

        /**
         * @brief Destroy the Receiver object
         *
         * @note close_socket() must be called in the child's destructor
         */
        virtual ~Receiver();

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
         * @brief Receive the latest data using UDP
         *
         * @param buffer Buffet to store data
         * @param buffer_size Size of the buffer where to store data
         *
         * @return size_t Number of bytes received
         */
        size_t receive_latest(char* buffer, const size_t buffer_size);

        /**
         * @brief Set wheter to enable any source or source specific multicast.
         *        True for specific source, false for any source, default is false.
         *
         * @param specific_source Whether to enable source specific or not.
         */
        void force_specific_source(bool specific_source);

        /**
         * @brief Set the receiver endpoint
         *
         * @param receiver_address Address where to send data
         * @param receiver_port Port where to send data
         *
         * @warning reset() must be called after changing the endpoint
         */
        void set_receiver_endpoint(const std::string receiver_address, const short receiver_port);

        /**
         * @brief Reset the receiver
         */
        void reset(void);

    protected:
        bool specific_source;  /**< True for specific source, false for any source, default is false */

        boost::asio::io_context io_context;   /**< boost/asio I/O execution context */
        boost::asio::ip::udp::socket* socket; /**< Network socket*/

        /**
         * @brief Endpoints: addresses and ports pairs
         */
        boost::asio::ip::udp::endpoint sender_endpoint;
        boost::asio::ip::udp::endpoint receiver_endpoint;

        /**
         * @brief Open the socket with the desired options
         *
         * @par Example:
         * @code{.cpp}
         *  // Create the socket so that multiple may be bound to the same address.
         *  this->socket->open(this->receiver_endpoint.protocol());
         *
         *  // Set socket options
         *
         *  // Bind to endpoint
         *  this->socket->bind(this->receiver_endpoint);
         *
         *  // Set blocking mode
         *  this->socket->non_blocking(true);
         * @endcode
         */
        virtual void open_socket() = 0;

        /**
         * @brief Close the socket
         */
        virtual void close_socket();
};
}  // namespace udp
}  // namespace travesim

#endif // __RECEIVER_H__
