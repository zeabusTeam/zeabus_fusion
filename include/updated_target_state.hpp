// FILE			: updated_target_state.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, January 07 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <iostream>

#include    <mutex>

#include    <ros/ros.h>

#include    <zeabus_utility/BoolArray6,hpp>

#include    <geometry_msgs/Pose.h>

#include    <geometry_msgs/Vector3.h>

#include    <tf/LinearMath/Matrix3x3.h>

#include    <tf/LinearMath/Quaternion.h>

#include    <zeabus/ros/convert/geometry_quaternion.hpp>

#include    <zeabus/ros/subscriber/base_class.hpp>

#ifndef _UPDATED_TARGET_STATE_HPP__
#define _UPDATED_TARGET_STATE_HPP__

class UpdatedTargetState
{

    public:
        UpdatedTargetState( ros::NodeHandle* ptr_node_handle );

        void setup_all_variable( geometry_msgs::Pose* ptr_message_target_state,
                std::mutex* ptr_lock_target,
                geometry_msgs::Pose* ptr_message_current_state,
                std::mutex* ptr_lock_current );

        void setup_subscriber( std::string topic_input , double timeout );

        void updated( ros::Time* time_stamp );

    protected:
        zeabus_utility::BoolArray6 message;
        std::mutex lock_mask;
        zeabus_ros::subscriber::BaseClass< zeabus_utility::BoolArray6 > listener_mask_updated;

        double timeout;

        ros::NodeHandle* ptr_node_handle;
        geometry_msgs::Pose* ptr_message_target_state;
        geometry_msgs::Pose* ptr_message_current_state;
        std::mutex* ptr_lock_target;
        std::mutex* ptr_lock_current;

};

#endif // _UPDATED_TARGET_STATE_HPP__
