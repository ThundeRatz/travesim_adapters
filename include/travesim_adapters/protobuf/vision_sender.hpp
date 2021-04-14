/**
 * @file vision_sender.hpp
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 * @brief Vision data sender with UDP and protobuf
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include "travesim_adapters/udp/multicast_sender.hpp"
#include "travesim_adapters/data/field_state.hpp"
#include "packet.pb.h"

#ifndef __VISION_SENDER_H__
#define __VISION_SENDER_H__

namespace travesim {
namespace proto {
/**
 * @brief Vision sender class with UDP and protobuf
 *
 */
class VisionSender {
    public:
        /**
         * @note The default construct may not be used
         *
         */
        VisionSender() = delete;

        /**
         * @brief Construct a new Vision Sender object
         *
         * @param multicast_address Vision multicast address
         * @param multicast_port Vision multicast port
         */
        VisionSender(const std::string multicast_address, const short multicast_port);

        /**
         * @brief Destroy the Vision Sender object
         *
         */
        ~VisionSender();

        /**
         * @brief Send vision data with UDP and protobuf
         *
         * @param p_field_state Pointer to field state to be sent
         */
        void send(FieldState* p_field_state);

        /**
         * @brief Convert a FieldState object to a Environment protobuf message object
         *
         * @param p_field_state Pointer to field state to be converted
         * @return fira_message::sim_to_ref::Environment
         */
        static fira_message::sim_to_ref::Environment field_state_to_env_pb_msg(FieldState* p_field_state);

    private:
        /**
         * @brief Pointer to UDP multicast sender
         *
         */
        udp::MulticastSender* multicast_sender;
};
}  // namespace proto
}  // namespace travesim

#endif // __VISION_SENDER_H__
