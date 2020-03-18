// FILE			: observer_integral.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <observer/observer.hpp>

boost::qvm::mat< double , 6 , 1 > mat_current_state;
boost::qvm::mat< double , 6 , 1 > mat_different_state;
boost::qvm::mat< double , 6 , 1 > mat_velocity;
boost::array< double , 6 > arr_robot_velocity;
boost::array< double , 3 > arr_odom_linear_velocity;
tf::Quaternion current_quaternion;

void reset_integral()
{
    observer_status  = false;
    mat_acceleration *= 0;
    mat_velocity *= 0;
    arr_robot_velocity.assign( 0 );
    arr_odom_linear_velocity.assign( 0 );
}

// Input of this function will have
//  mat_acceleration in robot frame
//  config about orientation and angular velocity
//  config about linear position and velocity in z
//  config position x and y include velocity from vision data
//  data about previous state
void active_integral( const double& observer_diff, 
        const double& localize_diff,
        const double& vision_diff )
{
    double temp_array[3];
    tf::Quaternion temp_quaternion;
    if( ! observer_status )
    {
        observer_data.pose = localize_data.pose;
        observer_data.twist = localize_data.twist;
        active_bound_limit(); // call for try active observer_status
        goto exit_active_integral;
    }

    // Now we have acceleration we have to calculate about next time velocity before
    // check config of position and velocity linear z
    std::memcpy( (void*) &mat_velocity.a[0][0],
            (void*) arr_robot_velocity.c_array(),
            sizeof( double ) * 6 );
    mat_velocity += mat_acceleration * observer_diff;
    // Now we have current velocity we have to calculate distance
    if( b_config_integral_z_linear )
    {
        b_config_integral_z_linear = false;
        active_bound_limit_depth();
    }

    if( b_config_integral_rotation )
    {
        b_config_integral_rotation = false;
        observer_data.pose.pose.orientation = localize_data.pose.pose.orientation;
    }
    else
    {
        temp_array[0] = boost::qvm::A30( mat_velocity ) * observer_diff;
        temp_array[1] = boost::qvm::A40( mat_velocity ) * observer_diff;
        temp_array[2] = boost::qvm::A50( mat_velocity ) * observer_diff;
        zeabus::math::inv_rotation( current_quaternion , temp_array );
    }

    if( b_config_integral_vision )
    {
        b_config_integral_vision = false;
        observer_data.pose.pose.position.x = vision_data.pose.pose.position.x + 
                boost::qvm::A00( mat_velocity ) * vision_diff -
                boost::qvm::A00( mat_acceleration ) * vision_diff / 2; 
        observer_data.pose.pose.position.y = vision_data.pose.pose.position.y + 
                boost::qvm::A00( mat_velocity ) * vision_diff -
                boost::qvm::A00( mat_acceleration ) * vision_diff / 2; 
    }
    else
    {
        observer_data.pose.pose.position.x += 
                boost::qvm::A00( mat_velocity ) * observer_diff -
                boost::qvm::A00( mat_acceleration ) * observer_diff / 2; 
        observer_data.pose.pose.position.y +=
                boost::qvm::A00( mat_velocity ) * observer_diff -
                boost::qvm::A00( mat_acceleration ) * observer_diff / 2; 
    }

    active_bound_limit(); // call for normal progress
exit_active_integral:
    return; 
}
