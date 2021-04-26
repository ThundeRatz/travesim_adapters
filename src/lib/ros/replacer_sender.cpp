/**
 * @file replacer_sender.cpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#include <ros/console.h>
#include "travesim_adapters/ros/replacer_sender.hpp"

// #define ERROR_LOG(msg) ROS_ERROR_

namespace travesim {
namespace ros_side {
ReplacerSender::ReplacerSender() {
    _nh = ros::NodeHandlePtr(new ros::NodeHandle());
    this->reconnect_service_client();

    /**
     * @brief Create gzclient for the first time
     */

    // this->gzclient = this->_nh->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state", true);
}

void ReplacerSender::reconnect_service_client() {
    /**
     * @brief If the connection dropped, we try to reconnect
     */
    if (!this->gzclient.isValid()) {
        this->gzclient = this->_nh->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    }
}

bool ReplacerSender::set_model_state(gazebo_msgs::ModelStatePtr model_state) {
    static gazebo_msgs::SetModelStateRequest request;
    static gazebo_msgs::SetModelStateResponse response;

    this->reconnect_service_client();
    request.model_state = (*model_state);

    bool status = this->gzclient.call(request, response);

    if (!(status && response.success)) {
        ROS_ERROR_STREAM("Error while calling set_model_state " << (*model_state));
        return false;
    }

    return true;
}

bool ReplacerSender::set_team_state(std::vector<gazebo_msgs::ModelStatePtr>* model_states) {
    for (uint32_t i = 0; i < model_states->size(); i++) {
        if (!this->set_model_state(model_states->at(i))) {
            ROS_ERROR_STREAM("Error while calling set_team_state at i = " << i);
            return false;
        }
    }

    return true;
}
}  // namespace ros_side
}  // namespace travesim
