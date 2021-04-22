/**
 * @file unicast_sender.hpp
 *
 * @author Lucas Haug <lucas.haug@thuneratz.org>
 * @author Lucas Schneider <lucas.schneider@thuneratz.org>

 * @brief Send data using UDP in unicast mode
 *
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 */

#include "travesim_adapters/udp/sender.hpp"

#ifndef __UNICAST_SENDER_H__
#define __UNICAST_SENDER_H__

namespace travesim {
namespace udp {
/**
 * @brief Sender class using UDP in unicast mode
 */
class UnicastSender : public Sender {
    public:
        /**
         * @brief Construct a new UnicastSender object
         *
         * @param receiver_address Address where to send data
         * @param receiver_port Port where to send data
         */
        Sender(const std::string receiver_address, const short receiver_port);
};
}  // namespace udp
}  // namespace travesim

#endif // __UNICAST_SENDER_H__
