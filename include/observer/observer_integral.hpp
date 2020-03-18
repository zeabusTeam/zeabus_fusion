// FILE			: observer_integral.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

extern boost::qvm::mat< double , 6 , 1 > mat_acceleration;
extern boost::qvm::mat< double , 6 , 1 > mat_velocity;
extern boost::array< double , 6 > arr_robot_velocity;
extern boost::array< double , 3 > arr_odom_linear_velocity;
extern tf::Quaternion current_quaternion;

#ifndef _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_INTEGRAL_HPP__
#define _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_INTEGRAL_HPP__
void reset_integral();
void active_integral( const double& observer_diff , 
        const double& localize_diff , 
        const double& vision_diff );
#endif // _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_INTEGRAL_HPP__
