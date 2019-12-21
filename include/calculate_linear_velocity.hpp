// FILE			: calculate_linear_velocity.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, December 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <ros/time.h>

#include    <sensor_msgs/Imu.h>

#include    <geometry_msgs/Vector3.h>

#include    <tf/LinearMath/Quaternion.h> 

#include    <tf/LinearMath/Matrix3x3.h>

#include    <zeabus/ros/convert/geometry_vector3.hpp>

// This function will use time stamp of ptr_input to get period of data and will use to calculate
//  velocity in angular term

class AccelerationHandle
{
    // Function constructor
    public:
        AccelerationHandle( tf::Quaternion rotation = tf::Quaternion( 0 , 0 , 0 , 1 ) );

        void setup_rotation_quaternion( tf::Quaternion rotation_imu );

        void reset_state();

        void updated( const ros::Time* stamp_time, 
                const geometry_msgs::Vector3* linear_acceleration );

        void get_velocity( geometry_msgs::Vector3* linear_velocity );

    protected:
        tf::Quaternion rotation;

        geometry_msgs::Vector3 linear_velocity;

        geometry_msgs::Vector3 linear_acceleration;

        ros::Time stamp_time;    

}; // class Acceleration
