// FILE			: observer_reupdate.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 09 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

extern std::vector< nav_msgs::Odometry > vec_reupdate_data;
extern ros::Publisher publisher_point;

#ifndef _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_REUPDATE_HPP__
#define _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_REUPDATE_HPP__ 
void setup_reupdate();
bool reupdate_position( const nav_msgs::Odometry& vision_data );
bool reupdate_calculate( const nav_msgs::Odometry& vision_data, const tf::Vector3& vec_velocity );
#endif  
