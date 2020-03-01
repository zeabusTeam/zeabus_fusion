// FILE			: calculate_angular_velocity.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, October 26 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <localize/calculate_angular_velocity.hpp>

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

    diff_quaternion = input_quaternion.inverse() * diff_quaternion * input_quaternion ;

    double frequency = 1.0/(ptr_input->header.stamp - collect_time_stamp).toSec();

    geometry_msgs::Vector3 data_output;
    tf::Matrix3x3( diff_quaternion ).getRPY( data_output.x , data_output.y , data_output.z );
    data_output.x *= frequency;
    data_output.y *= frequency;
    data_output.z *= frequency;
    *ptr_output = data_output;
    zeabus_ros::convert::geometry_quaternion::tf( &(ptr_input->orientation) , &collect_quaternion );
    collect_time_stamp = ptr_input->header.stamp;
}

void calculate_angular_velocity( tf::Quaternion* ptr_input ,
        geometry_msgs::Vector3* ptr_output , ros::Time time_stamp  )
{
    static ros::Time collect_time_stamp = ros::Time::now();
    static tf::Quaternion collect_quaternion = tf::Quaternion( 0 , 0 , 0 , 1 );

    tf::Quaternion diff_quaternion = (*ptr_input) * collect_quaternion.inverse();
    double frequency =  1.0/( time_stamp - collect_time_stamp).toSec() ;

    // For look how to get RPY
//    tf::Matrix3x3( collect_quaternion ).getRPY( temp.x , temp.y , temp.z );

    geometry_msgs::Vector3 data_output;
    diff_quaternion = ptr_input->inverse() * diff_quaternion * (*ptr_input) ;
    tf::Matrix3x3( diff_quaternion ).getRPY( data_output.x , data_output.y , data_output.z );
    data_output.x *= frequency;
    data_output.y *= frequency;
    data_output.z *= frequency;
    *ptr_output = data_output;
    collect_quaternion = *ptr_input;
    collect_time_stamp = time_stamp;
}
