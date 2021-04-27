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
enum simulation_command_t {
    PAUSE,
    RESUME,
    RESET_WORLD,
    RESET_SIMULATION
};

typedef std::vector<gazebo_msgs::ModelStatePtr> state_vector_t;

class ReplacerSender {
    private:
        ros::NodeHandlePtr _nh;

        ros::ServiceClient gzclient;

    public:
        ReplacerSender();

        bool set_models_state(state_vector_t* model_states);

        bool set_model_state(gazebo_msgs::ModelStatePtr model_state);

        bool send_command(simulation_command_t command);

        void reconnect_service_client();
};
}  // namespace ros_side
}  // namespace travesim

#endif // __REPLACER_SENDER_H__
