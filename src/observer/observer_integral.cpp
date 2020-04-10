// FILE			: observer_integral.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
// #define _PRINT_ROBOT_STATE_

// MACRO CONDITION

#include    <observer/observer.hpp>

boost::qvm::mat< double , 6 , 1 > mat_current_state;
boost::qvm::mat< double , 6 , 1 > mat_different_state;
boost::qvm::mat< double , 6 , 1 > mat_velocity;
boost::array< double , 6 > arr_robot_velocity;
boost::array< double , 3 > arr_odom_linear_velocity;
boost::array< double , 3 > arr_odom_linear_acceleration;

void reset_integral()
{
    observer_status  = false;
    mat_acceleration *= 0;
    mat_velocity *= 0;
    arr_robot_velocity.assign( 0 );
    arr_odom_linear_velocity.assign( 0 );
    arr_odom_linear_acceleration.assign( 0 );
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
//    printf( "INTEGRAL OBSERVER : time report %15.3f%15.3f%15.3f\n" ,
//          observer_diff , localize_diff , vision_diff );
    double temp_array[3];
    tf::Quaternion temp_quaternion;
    if( ! observer_status )
    {
        reset_buffer_observer();
        observer_data.pose = localize_data.pose;
        observer_data.twist = localize_data.twist;
        active_bound_limit(); // call for try active observer_status
        goto exit_active_integral;
    }

    // Now we have acceleration we have to calculate about next time velocity before
    // check config of position and velocity linear z
    std::memcpy( (void*) &mat_velocity.a[0][0],
            (void*) arr_robot_velocity.c_array(),
            sizeof( double ) * 6 ); // can copy directly because observer model have use this var
    
    mat_velocity += ( mat_acceleration * observer_diff ); // calculate new velocity

//    mat_velocity.a[2][0] = 0;
//    mat_acceleration.a[2][0] = 0;

    std::memcpy( (void*) arr_odom_linear_acceleration.c_array(),
            (void*) &mat_acceleration.a[0][0],
            sizeof( double ) * 3 ); // copy acceleration linear

    std::memcpy( (void*) arr_odom_linear_velocity.c_array(),
            (void*) &mat_velocity.a[0][0],
            sizeof( double ) * 3 ); // copy velocity from robot to odom 

#ifdef _PRINT_ROBOT_STATE_
    printf( "INTEGRAL OBSERVER : mat_acceleration\n" );
    zeabus_boost::printT( mat_acceleration );
    printf( "INTEGRAL OBSERVER : mat_velocity\n" );
    zeabus_boost::printT( mat_velocity );
#endif // _PRINT_ROBOT_STATE_

    zeabus::math::inv_rotation( current_quaternion , arr_odom_linear_velocity.c_array() );
    zeabus::math::inv_rotation( current_quaternion , arr_odom_linear_acceleration.c_array() );

//=========> STEP : calculate position z
    // Now we have current velocity we have to calculate distance
    if( b_config_integral_z_linear )
    {
        b_config_integral_z_linear = false;
        observer_data.pose.pose.position.z += 
                boost::qvm::A20( mat_velocity ) * localize_diff - 
                boost::qvm::A20( mat_acceleration ) * pow( localize_diff , 2 ) / 2;
        active_bound_limit_depth();
    }
    else
    {
        observer_data.pose.pose.position.z += 
                arr_odom_linear_velocity[2] * observer_diff - 
                arr_odom_linear_acceleration[2] * pow( observer_diff , 2 ) / 2;
        active_bound_limit_depth();
    }

//=========> STEP : calculate orientation
    // Differential is on time because current quaternion will fix already use localize or estimate 
    if( b_config_integral_rotation )
    {
        b_config_integral_rotation = false;
        temp_array[0] = boost::qvm::A30( mat_velocity ) * localize_diff -
                boost::qvm::A30( mat_acceleration ) * pow( localize_diff , 2 ) / 2;
        temp_array[1] = boost::qvm::A40( mat_velocity ) * localize_diff -
                boost::qvm::A40( mat_acceleration ) * pow( localize_diff , 2 ) / 2;
        temp_array[2] = boost::qvm::A50( mat_velocity ) * localize_diff -
                boost::qvm::A50( mat_acceleration ) * pow( localize_diff , 2 ) / 2;
        zeabus::math::get_quaternion( temp_array[0] , temp_array[1] , temp_array[2] ,
                &temp_quaternion );
        zeabus::math::inv_rotation( current_quaternion , temp_quaternion );
        temp_quaternion = temp_quaternion * current_quaternion;
    }
    else
    {
        temp_array[0] = boost::qvm::A30( mat_velocity ) * observer_diff -
                boost::qvm::A30( mat_acceleration ) * pow( observer_diff , 2 ) / 2;
        temp_array[1] = boost::qvm::A40( mat_velocity ) * observer_diff -
                boost::qvm::A40( mat_acceleration ) * pow( observer_diff , 2 ) / 2;
        temp_array[2] = boost::qvm::A50( mat_velocity ) * observer_diff -
                boost::qvm::A50( mat_acceleration ) * pow( observer_diff , 2 ) / 2;
        zeabus::math::get_quaternion( temp_array[0] , temp_array[1] , temp_array[2] ,
                &temp_quaternion );
        zeabus::math::inv_rotation( current_quaternion , temp_quaternion );
        temp_quaternion = temp_quaternion * current_quaternion;
    }

//=========> STEP : calculate position x and y
    if( b_config_integral_vision )
    {
        std::cout   << "INTEGRAL OBSERVER : Use vision data position\n";
        b_config_integral_vision = false;
        observer_data.pose.pose.position.x = vision_data.pose.pose.position.x + 
                arr_odom_linear_velocity[0] * vision_diff -
                arr_odom_linear_acceleration[0] * pow( vision_diff , 2 ) / 2; 
        observer_data.pose.pose.position.y = vision_data.pose.pose.position.y + 
                arr_odom_linear_velocity[1] * vision_diff -
                arr_odom_linear_acceleration[1] * pow( vision_diff , 2 ) / 2; 
    }
    else
    {
        observer_data.pose.pose.position.x += 
                arr_odom_linear_velocity[0] * observer_diff -
                arr_odom_linear_acceleration[0] * pow( observer_diff , 2 ) / 2; 
        observer_data.pose.pose.position.y +=
                arr_odom_linear_velocity[1] * observer_diff -
                arr_odom_linear_acceleration[1] * pow( observer_diff , 2 ) / 2; 
#ifdef _PRINT_ROBOT_STATE_
        printf( "Observer add position x y are %11.6f , %11.6f\n\n" ,
                arr_odom_linear_velocity[0] * observer_diff -
                arr_odom_linear_acceleration[0] * pow( observer_diff , 2 ) / 2,
                arr_odom_linear_velocity[1] * observer_diff -
                arr_odom_linear_acceleration[1] * pow( observer_diff , 2 ) / 2 );
#endif
    }

    active_bound_limit(); // call for normal progress
    // Now we have to prepare answer to observer_data
    // copy rotation to message
    zeabus_ros::convert::geometry_quaternion::tf( &temp_quaternion , 
            &observer_data.pose.pose.orientation );
    // copy velocity to message
    observer_data.twist.twist.linear.x = boost::qvm::A00( mat_velocity );
    observer_data.twist.twist.linear.y = boost::qvm::A10( mat_velocity );
    observer_data.twist.twist.linear.z = boost::qvm::A20( mat_velocity );
    observer_data.twist.twist.angular.x = boost::qvm::A30( mat_velocity );
    observer_data.twist.twist.angular.y = boost::qvm::A40( mat_velocity );
    observer_data.twist.twist.angular.z = boost::qvm::A50( mat_velocity );

    // Now we have all data next we have to save odom velocity
    
exit_active_integral:
    return; 
}
