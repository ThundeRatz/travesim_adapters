/**
 * @file configurers_utils.cpp
 *
 * @author Lucas Haug <lucas.haug@thuneratz.org>
 *
 * @brief Configurers utilitary funtions
 *
 * @date 06/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include "travesim_adapters/configurers/configurers_utils.hpp"

/*****************************************
 * Private Constants
 *****************************************/

#define IPV4_NUM_OF_BYTES 4U

/*****************************************
 * Public Funtions Bodies Definitions
 *****************************************/

namespace travesim {
bool ipv4_string_to_uint(std::string ip_string, uint* ip_uint) {
    std::stringstream ip_stream(ip_string);

    char dot;

    if (ip_stream.rdbuf()->in_avail() == 0) {
        return false;
    } else {
        ip_stream >> ip_uint[0];

        if (ip_stream.fail()) {
            return false;
        }
    }

    for (uint i = 1; i < IPV4_NUM_OF_BYTES; i++) {
        if (ip_stream.rdbuf()->in_avail() == 0) {
            return false;
        } else {
            ip_stream >> dot;

            if (ip_stream.fail()) {
                return false;
            }

            if (dot != '.') {
                return false;
            }
        }

        if (ip_stream.rdbuf()->in_avail() == 0) {
            return false;
        } else {
            ip_stream >> ip_uint[i];

            if (ip_stream.fail()) {
                return false;
            }
        }
    }

    if (ip_stream.rdbuf()->in_avail() != 0) {
        return false;
    }

    return true;
}

bool check_valid_ip(std::string ip, std::string min_ip, std::string max_ip) {
    uint ip_uint[IPV4_NUM_OF_BYTES] = {0};
    uint min_ip_uint[IPV4_NUM_OF_BYTES] = {0};
    uint max_ip_uint[IPV4_NUM_OF_BYTES] = {0};

    if (!ipv4_string_to_uint(ip, ip_uint)) {
        return false;
    }

    if (!ipv4_string_to_uint(min_ip, min_ip_uint)) {
        return false;
    }

    if (!ipv4_string_to_uint(max_ip, max_ip_uint)) {
        return false;
    }

    for (uint i = 0; i < IPV4_NUM_OF_BYTES; i++) {
        if (ip_uint[i] > 0xFF) {
            return false;
        }

        if (ip_uint[i] > max_ip_uint[i]) {
            return false;
        }

        if (ip_uint[i] < min_ip_uint[i]) {
            return false;
        }
    }

    return true;
}
}
