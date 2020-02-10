// FILE			: mapper.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 07 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <mutex>

#include    <iostream>

#include    <nav_msgs/Odometry.h>

#include    <zeabus/ros/node.hpp>

#include    <zeabus/ros/convert/geometry_quaternion.hpp>

#include    <zeabus/ros/subscriber/base_class.hpp>

#include    <zeabus/ros/path_file.hpp>

#include    <zeabus/ros/static_tf.hpp>

#include    <tf/LinearMath/Quaternion.h>

#include    <tf/transform_broadcaster.h>

#include    <tf/transform_listener.h>

#include    <ros/ros.h>

#include    <zeabus_utility/Float64Array8.h>

#include    <zeabus/robot.hpp>

#include    <zeabus/ros/dynamic_reconfigure.hpp>

#include    <boost/qvm/vec_access.hpp>

#include    <boost/qvm/mat_access.hpp> 

#include    <zeabus/file.hpp>

#include    <calculate_by_force.hpp>

#include    <geometry_msgs/TransformStamped.h>

#include    <mapper_reconfigure.hpp>

extern double mass;
extern double moment_inertia;
extern double volumn;
extern double viscosty[ 6 ];

extern bool avaliable_new_parameter;
extern std::mutex lock_dynamic_reconfigure;
