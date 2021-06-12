/**
 * @file replacer_configurer.hpp
 *
 * @author Lucas Haug <lucas.haug@thuneratz.org>
 *
 * @brief Configurer for the Replacer
 *
 * @date 06/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/recursive_mutex.hpp>
#include <travesim_adapters/ReplacerConfig.h>

#ifndef __REPLACER_CONFIGURER_H__
#define __REPLACER_CONFIGURER_H__

namespace travesim {
/**
 * @brief ReplacerConfigurer class definition
 */

class ReplacerConfigurer {
    public:
        /**
         * @brief Construct a new ReplacerConfigurer object
         */
        ReplacerConfigurer(void);

        /**
         * @brief Get the replacer configured address
         *
         * @return std::string address string
         */
        std::string get_address(void);

        /**
         * @brief Get the replacer configured port
         *
         * @return uint16_t port number
         */
        uint16_t get_port(void);

        /**
         * @brief Get the specific_source config
         *
         * @return true if specific source is enabled, false otherwise
         */
        bool get_specific_source(void);

        /**
         * @brief Get the reset config
         *
         * @return true if should reset, false otherwise
         */
        bool get_reset(void);

        /**
         * @brief Output stream operator overloading
         */
        friend std::ostream& operator <<(std::ostream& output, const ReplacerConfigurer& repl_conf);

    private:
        /**@{*/
        /** Configurer private types */
        typedef travesim_adapters::ReplacerConfig config_t;     /**< Replacer config type */
        typedef dynamic_reconfigure::Server<config_t> server_t; /**< Replacer server type */
        /**@}*/

        bool reconfigured;  /**< Whether the configs where changed or not */

        config_t config;  /**< Current config */

        std::unique_ptr<ros::NodeHandle> node_handle;  /**< Pointer to ROS Node Handle */

        boost::recursive_mutex mutex;     /**< Dynamic Reconfigure Server Mutex for thread-safety */
        std::unique_ptr<server_t> server; /**< Dynamic Reconfigure Server */

        /**
         * @brief Dynamic Reconfigure Server Callback
         *
         * @param config New config received
         * @param level Result of doing an "OR" operation between all of
         *              level values of the parameters that have changed
         */
        void callback(config_t& config, uint32_t level);
};
}  // namespace travesim

#endif // __REPLACER_CONFIGURER_H__
