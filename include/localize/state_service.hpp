// FILE			: state_service.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 01 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <iostream>
#include    <ros/ros.h>

#include    <mutex>
#include    <cmath>

#include    <nav_msgs/Odometry.h>

#include    <zeabus_utility/StateConnection.h>

#ifndef _LOCALIZE_STATE_SERVICE_HPP__
#define _LOCALIZE_STATE_SERVICE_HPP__

class StateService
{

    public:

        StateService( ros::NodeHandle* ptr_node_handle );

        void setup_all_variable( nav_msgs::Odometry* ptr_message_current_state,
                std::mutex* ptr_lock_current );

        void setup_all_service();

        ros::ServiceServer service_vision_state;
        bool callback_vision_service( zeabus_utility::StateConnection::Request& request,
                zeabus_utility::StateConnection::Response& response );

    protected:
        ros::NodeHandle* ptr_node_handle;

        nav_msgs::Odometry* ptr_message_current_state;
        std::mutex* ptr_lock_current;
}; 

#endif // _LOCALIZE_STATE_SERVICE_HPP__ 
