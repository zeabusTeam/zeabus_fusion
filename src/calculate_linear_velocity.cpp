// FILE			: calculate_linear_velocity.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, October 26 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <calculate_linear_velocity.hpp>

AccelerationHandle::AccelerationHandle( tf::Quaternion rotation )
{
    this->setup_rotation_quaternion( rotation );
    this->reset_state();
} // function constructor

void AccelerationHandle::setup_rotation_quaternion( tf::Quaternion rotation )
{
    this->rotation = rotation;
}

void AccelerationHandle::reset_state()
{
    this->linear_velocity.x = 0;
    this->linear_velocity.y = 0;
    this->linear_velocity.z = 0;
    this->linear_acceleration.x = 0;
    this->linear_acceleration.y = 0;
    this->linear_acceleration.z = 0;
    ros::Time stamp_time = ros::Time::now();
} // function reset_state

// equation v_1 = v_0 + (a_0 + a_1 ) / 2 * t
void AccelerationHandle::updated( const ros::Time* stamp_time, 
        const geometry_msgs::Vector3* linear_acceleration )
{
    tf::Quaternion robot_linear_velocity;
    zeabus_ros::convert::geometry_vector3::tf( linear_acceleration , &robot_linear_velocity );
    robot_linear_velocity = this->rotation.inverse() * 
            robot_linear_velocity * 
            this->rotation;
    double multiple = ( *stamp_time - this->stamp_time ).toSec() / 2.0;
    this->linear_velocity.x += ( this->linear_acceleration.x + linear_acceleration->x ) * multiple; 
    this->linear_velocity.y += ( this->linear_acceleration.y + linear_acceleration->y ) * multiple; 
    this->linear_velocity.z += ( this->linear_acceleration.z + linear_acceleration->z ) * multiple;
    // Updated local varible
    this->stamp_time = *stamp_time;
    this->linear_acceleration = *linear_acceleration;
} // function updated

void AccelerationHandle::get_velocity( geometry_msgs::Vector3* linear_velocity )
{
    *linear_velocity = this->linear_velocity;
} // function get_velocity

