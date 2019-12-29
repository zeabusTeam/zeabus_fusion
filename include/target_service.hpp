// FILE			: target_service.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, October 30 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <iostream>

#include    <ros/ros.h>

#include    <mutex>

#include    <geometry_msgs/Pose.h>

#include    <geometry_msgs/Vector3.h>

#include    <tf/LinearMath/Matrix3x3.h>

#include    <tf/LinearMath/Quaternion.h> 

#include    <zeabus_utility/SendFloat.h>

#include    <zeabus/ros/convert/geometry_quaternion.hpp>

#ifndef _TARGET_SERVICE_HPP__
#define _TARGET_SERVICE_HPP__

class TargetService
{

    public:

        TargetService( ros::NodeHandle* ptr_node_handle );

        void setup_all_variable( bool* ptr_updated_target_state,
                geometry_msgs::Pose* ptr_message_target_state,
                std::mutex* ptr_lock );

        void setup_all_service();    
        
        ros::ServiceServer service_absolute_depth;
        bool callback_absolute_depth( zeabus_utility::SendFloat::Request& request,
                zeabus_utility::SendFloat::Response& response );

        ros::ServiceServer service_relative_depth;
        bool callback_relative_depth( zeabus_utility::SendFloat::Request& request,
                zeabus_utility::SendFloat::Response& response );

        ros::ServiceServer service_relative_yaw;
        bool callback_relative_yaw( zeabus_utility::SendFloat::Request& request,
                zeabus_utility::SendFloat::Response& response );

        ros::ServiceServer service_relative_roll;
        bool callback_relative_roll( zeabus_utility::SendFloat::Request& request,
                zeabus_utility::SendFloat::Response& response );

        ros::ServiceServer service_relative_pitch;
        bool callback_relative_pitch( zeabus_utility::SendFloat::Request& request,
                zeabus_utility::SendFloat::Response& response );

        ros::ServiceServer service_absolute_yaw;
        bool callback_absolute_yaw( zeabus_utility::SendFloat::Request& request,
                zeabus_utility::SendFloat::Response& response );

        ros::ServiceServer service_absolute_roll;
        bool callback_absolute_roll( zeabus_utility::SendFloat::Request& request,
                zeabus_utility::SendFloat::Response& response );

        ros::ServiceServer service_absolute_pitch;
        bool callback_absolute_pitch( zeabus_utility::SendFloat::Request& request,
                zeabus_utility::SendFloat::Response& response );

    protected:
        bool* ptr_updated_target_state;
        void updated_target_state();
        geometry_msgs::Pose* ptr_message_target_state;
        std::mutex* ptr_lock;
        ros::NodeHandle* ptr_node_handle;
};

#endif
