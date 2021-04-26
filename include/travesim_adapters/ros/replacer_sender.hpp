/**
 * @file replacer_sender.hpp
 * @author Felipe Gomes de Melo <felipe.gomes@thunderatz.org>
 * @brief
 * @date 04/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 *
 */

#ifndef __REPLACER_SENDER_H__
#define __REPLACER_SENDER_H__

#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>

namespace travesim {
namespace ros_side {
class ReplacerSender {
    private:
        ros::NodeHandlePtr _nh;

        ros::ServiceClient gzclient;

    public:
        ReplacerSender();

        bool set_team_state(std::vector<gazebo_msgs::ModelStatePtr>* model_states);

        bool set_model_state(gazebo_msgs::ModelStatePtr model_state);

        void reconnect_service_client();
};
}  // namespace ros_side
}  // namespace travesim

#endif // __REPLACER_SENDER_H__
