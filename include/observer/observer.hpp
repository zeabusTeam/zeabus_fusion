// FILE			: observer.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 09 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <cmath>
#include    <mutex>
#include    <vector>
#include    <cstring>
#include    <cstdlib>
#include    <iostream>

#include    <zeabus/file.hpp> // Read file about parameter 
#include    <zeabus/robot.hpp>
#include    <zeabus/ros/node.hpp> // Activate other thread receive data in ros system
#include    <zeabus/escape_code.hpp>
#include    <zeabus/ros/path_file.hpp> // Find path file in ROS system
#include    <zeabus/math/quaternion.hpp> // zeabus handle quaternion
#include    <zeabus/math/boost/operations.hpp> // include concat function
#include    <zeabus/math/boost/print_data.hpp> // include print matrix and vec in Boost qvm
#include    <zeabus/ros/convert/nav_odometry.hpp>
#include    <zeabus/ros/subscriber/base_class.hpp> // include template about listen
#include    <zeabus/ros/convert/geometry_vector3.hpp>
#include    <zeabus/ros/convert/geometry_quaternion.hpp>

#include    <tf/LinearMath/Quaternion.h>
#include    <tf/transform_listener.h>
#include    <tf/transform_broadcaster.h>

#include    <ros/ros.h>
#include    <std_msgs/String.h>
#include    <nav_msgs/Odometry.h>
#include    <zeabus_utility/Float64Array8.h>
#include    <zeabus_utility/Float64Array6.h>

#include    <boost/array.hpp> // Beacause ros msg fix length will use boost array

#include    <boost/qvm/vec_access.hpp>
#include    <boost/qvm/mat_access.hpp>
#include    <boost/qvm/vec_operations.hpp>
#include    <boost/qvm/mat_operations.hpp>

#include    <observer/observer_parameter.hpp>
#include    <observer/observer_localize.hpp>
#include    <observer/observer_vision.hpp>
#include    <observer/observer_model.hpp>
#include    <observer/observer_bound_limit.hpp>
#include    <observer/observer_integral.hpp>

extern nav_msgs::Odometry localize_data;
extern nav_msgs::Odometry observer_data;
extern nav_msgs::Odometry vision_data;
extern ros::Publisher observer_publisher;
extern boost::qvm::mat< double , 6 , 1 > mat_force_thruster;
extern tf::Quaternion current_quaternion;

extern const double global_time_limit_buffer; 
extern std::vector< nav_msgs::Odometry > vec_observer_data;

#ifndef _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_HPP__
#define _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_HPP__
unsigned int vision_stamp_handle( const ros::Time stamp );
unsigned int localize_stamp_handle( const ros::Time stamp );
void publish( const std::string message );
bool reupdate_position( const nav_msgs::Odometry& vision_data );
void check_buffer_observer( const ros::Time& minimum_time );
#endif // _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_HPP__
