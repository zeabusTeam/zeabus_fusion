// FILE			: calculate_angular_velocity.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, October 26 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <cmath>

#include    <ros/time.h>

#include    <geometry_msgs/Vector3.h>

#include    <sensor_msgs/Imu.h>

#include    <tf/LinearMath/Quaternion.h> 

#include    <tf/LinearMath/Matrix3x3.h>

#include    <zeabus/ros/convert/geometry_quaternion.hpp>

// This function will use time stamp of ptr_input to get period of data and will use to calculate
//  velocity in angular term
void calculate_angular_velocity( sensor_msgs::Imu* ptr_input , 
        geometry_msgs::Vector3* ptr_output ); 

void calculate_angular_velocity( tf::Quaternion* ptr_input , 
        geometry_msgs::Vector3* ptr_output,
        ros::Time time_stamp ); 
