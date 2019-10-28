// FILE			: calculate_angular_velocity.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, October 26 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <calculate_angular_velocity.hpp>

void calculate_angular_velocity( sensor_msgs::Imu* ptr_input ,
        geometry_msgs::Vector3* ptr_output )
{
    static ros::Time collect_time_stamp = ros::Time::now();
    static tf::Quaternion collect_quaternion = tf::Quaternion( 0 , 0 , 0 , 1 );
    tf::Quaternion input_quaternion = tf::Quaternion( ptr_input->orientation.x,
            ptr_input->orientation.y,
            ptr_input->orientation.z,
            ptr_input->orientation.w );
    tf::Quaternion diff_quaternion = input_quaternion * collect_quaternion.inverse();

    double frequency = 1.0/(ptr_input->header.stamp - collect_time_stamp).toSec();

    geometry_msgs::Vector3 data_output;
    tf::Matrix3x3( diff_quaternion ).getRPY( data_output.x , data_output.y , data_output.z );
    data_output.x *= frequency;
    data_output.y *= frequency;
    data_output.z *= frequency;
    *ptr_output = data_output;
}
