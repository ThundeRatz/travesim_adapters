/**
 * @file configurers_utils.hpp
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

#include <string>
#include <sstream>

/*****************************************
 * Public Funtions Prototypes
 *****************************************/

namespace travesim {
/**
 * @brief Converts a IPv4 in a string to a array of unsigned integers,
 *        where the most significant byte of the IP address is in the
 *        position 0 and the last significant in the position 3.
 *
 * @note Expects IPv4 in the quad-dotted notation.
 *
 * @param ip_string IPv4 string to be converted
 * @param ip_uint Pointer where to store the converted IPv4
 *
 * @return true if the conversion was successful, false otherwise
 */
bool ipv4_string_to_uint(std::string ip_string, uint* ip_uint);

/**
 * @brief Checks if a IP is valid and is in the specified range
 *
 * @note Expects IPv4 in the quad-dotted notation.
 *
 * @param ip IP string to be converted
 * @param min_ip Minimum IP interval value
 * @param max_ip Maximum IP interval value
 *
 * @return true if the IP is valid, false otherwise
 */
bool check_valid_ip(std::string ip, std::string min_ip, std::string max_ip);
}  // namespace travesim
