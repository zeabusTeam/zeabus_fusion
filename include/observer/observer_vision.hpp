// FILE			: observer_vision.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 09 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

extern bool b_config_integral_vision;
extern bool b_config_parameter_vision;
extern const double limit_time_vision;

extern tf::Vector3 vec_vision_velocity_1;
extern tf::Vector3 vec_vision_velocity_2;
extern tf::Vector3 vec_vision_acceleration;

struct DataRosVision
{
    ros::Time stamp;
    tf::Vector3 pose;
    tf::Quaternion quaternion;

    DataRosVision( const nav_msgs::Odometry* data )
    {
        this->stamp = data->header.stamp;
        zeabus_ros::convert::geometry_quaternion::tf( data , &( this->quaternion ) );
        this->pose = tf::Vector3( data->pose.pose.position.x,
                data->pose.pose.position.y,
                data->pose.pose.position.z );
    }
};

extern std::vector< DataRosVision > vec_vision_data;

#ifndef _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_VISION_HPP__
#define _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_VISION_HPP__
void reset_vision();
void active_vision();
void check_buffer_vision();
#endif // _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_VISION_HPP__
