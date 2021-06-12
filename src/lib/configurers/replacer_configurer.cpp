/**
 * @file replacer_configurer.cpp
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

#include "travesim_adapters/configurers/replacer_configurer.hpp"

/*****************************************
 * Private Constants
 *****************************************/

#define REPLACER_CONFIGURER_NAMESPACE "replacer"

namespace travesim {
/*****************************************
 * Public Methods Bodies Definitions
 *****************************************/

ReplacerConfigurer::ReplacerConfigurer(void) {
    this->reconfigured = false;

    this->node_handle = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle(REPLACER_CONFIGURER_NAMESPACE));

    this->server = std::unique_ptr<server_t>(new server_t(this->mutex, *(this->node_handle)));

    server_t::CallbackType server_cb;
    server_cb = boost::bind(&ReplacerConfigurer::callback, this, _1, _2);
    this->server->setCallback(server_cb);

    // Update the default config in the dynamic reconfigurer
    boost::recursive_mutex::scoped_lock scoped_lock(this->mutex);
    this->server->getConfigDefault(this->config);
    this->server->updateConfig(this->config);
}

std::string ReplacerConfigurer::get_address(void) {
    return this->config.replacer_address;
}

uint16_t ReplacerConfigurer::get_port(void) {
    return this->config.replacer_port;
}

bool ReplacerConfigurer::get_specific_source(void) {
    return this->config.specific_source;
}

bool ReplacerConfigurer::get_reset(void) {
    bool should_reset = this->config.reset || this->reconfigured;

    if (should_reset) {
        boost::recursive_mutex::scoped_lock scoped_lock(this->mutex);

        this->reconfigured = false;

        this->config.reset = this->reconfigured;

        this->server->updateConfig(this->config);

        return true;
    }

    return false;
}

std::ostream& operator <<(std::ostream& output, const ReplacerConfigurer& repl_conf) {
    output << "Replacer Address: " << repl_conf.config.replacer_address << std::endl;
    output << "Replacer Port: " << repl_conf.config.replacer_port << std::endl;
    output << "Specific Source: " << (repl_conf.config.specific_source ? "True" : "False") << std::endl;
    output << "Reset: " << ((repl_conf.config.reset || repl_conf.reconfigured) ? "True" : "False") << std::endl;

    return output;
}

/*****************************************
 * Private Methods Bodies Definitions
 *****************************************/

void ReplacerConfigurer::callback(travesim_adapters::ReplacerConfig& config, uint32_t level) {
    this->config = config;
    this->reconfigured = true;
}
}
