// FILE			: observer_localize.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 09 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <observer/observer.hpp>

bool b_config_model_z_linear = false;
bool b_config_model_rotation = false;
bool b_config_integral_rotation = false;
bool b_config_integral_z_linear = false;
bool b_start_active_z = false;
bool b_start_active_rotation = false;
double mean_z = 0;
double max_z = 0;
double min_z = 0;

const unsigned int ui_limit_rotation = 2;
unsigned int ui_count_rotation =0;
tf::Vector3 vec_angular_velocity_1;
tf::Vector3 vec_angular_velocity_2;
tf::Vector3 vec_angular_acceleration;

void active_localize()
{
    active_rotation();
    active_z();
} 

// In rotation we assum data of imu have vibration
// We will accept only data have result same direction will equation
// But result about velocity will accept to config model becuase 
// We can't to decision that situation it not real 100 percent
void active_rotation()
{
    // Decision on yaw term
    static ros::Time data_stamp = ros::Time::now();
    if( ! b_start_active_rotation )
    {
        b_config_integral_rotation = true;
        ui_count_rotation = 0;
        observer_data.pose.pose.orientation = localize_data.pose.pose.orientation;
        observer_data.twist.twist.angular = localize_data.twist.twist.angular;
        zeabus_ros::convert::geometry_vector3::tf( 
                &observer_data.twist.twist.angular,
                &vec_angular_velocity_1 );
        data_stamp = localize_data.header.stamp;
        goto exit_active_rotation;
    }

    // First time to this function get about sign of yaw orientation
    if( localize_data.twist.twist.angular.z > 0 )
    {
        if( arr_robot_velocity[ 5 ] > 0 )
        {
            b_config_integral_rotation = true;
            ui_count_rotation = 0;
        }
        else
        {
            ui_count_rotation++;
        }
    } // now sensor tell twist angular z is position
    else
    {
        if( arr_robot_velocity[ 5 ]  < 0 )
        {
            b_config_integral_rotation = true;
            ui_count_rotation = 0;
        }
        else
        {
            ui_count_rotation++;
        }
    }

    if( ui_count_rotation > ui_limit_rotation )
    {
        b_config_integral_rotation = true;
        ui_count_rotation = 0;
    }

    zeabus_ros::convert::geometry_vector3::tf( 
            &localize_data.twist.twist.angular,
            &vec_angular_velocity_2 );

    vec_angular_acceleration = ( vec_angular_velocity_2 - vec_angular_velocity_1 ) /
            ( localize_data.header.stamp - data_stamp ).toSec();
    b_config_model_rotation = true;
    vec_angular_velocity_1 = vec_angular_velocity_2;
exit_active_rotation:
    return;
} // function active_rotation 

void reset_rotation()
{
    b_config_integral_rotation = false;
    b_config_model_rotation = false;
    b_start_active_rotation = false;
    ui_count_rotation = 0;
} // function reset_rotation

// In data you ses in function active_z
//  Data this function is use to decide credit of data
void active_z( )
{
    if( ! b_start_active_z )
    {
        min_z = localize_data.pose.pose.position.z - global_diff_z_half;
        max_z = localize_data.pose.pose.position.z + global_diff_z_half;
        mean_z = localize_data.pose.pose.position.z;
        observer_data.pose.pose.position.z = localize_data.pose.pose.position.z;
        b_start_active_z = true;
        goto exit_active_z;
    }
    
    if( fabs( localize_data.pose.pose.position.z - mean_z ) > global_diff_z * 2 )
    { // filter this value because estimate this is noise
        goto exit_active_z;
    }
    else
    {
        b_config_integral_z_linear = true;
        mean_z = localize_data.pose.pose.position.z;
        max_z = mean_z + global_diff_z_half;
        min_z = mean_z - global_diff_z_half;
    }
exit_active_z:
    return;
}

void reset_localize()
{
    reset_z();
    reset_rotation();
}

void reset_z()
{
    b_config_integral_z_linear = false;
    b_config_model_z_linear = false;
    b_start_active_z = false;
    mean_z = 0;
    max_z = 0;
    min_z = 0;
}
