// FILE			: localize.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, October 24 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <mutex>

#include    <iostream>

#include    <sensor_msgs/Imu.h>

#include    <nav_msgs/Odometry.h>

#include    <zeabus_utility/HeaderFloat64.h>

#include    <zeabus/ros/node.hpp>

#include    <zeabus/ros/subscriber/base_class.hpp>

#include    <tf/LinearMath/Quaternion.h>

#include    <tf/transform_broadcaster.h>

#include    <calculate_angular_velocity.hpp>

int main( int argv , char** argc )
{
    zeabus_ros::Node node( argv , argc , "localize" );

    ros::NodeHandle ph("~"); // for param node handle
    ros::NodeHandle nh(""); // general node handle

    node.spin();
    // This frequency is tell you about frequency to calculate new state of robot
    int frequency;
    ph.param< int >( "frequency" , frequency , 50 );    

    std::string topic_imu;
    ph.param< std::string >( "topic_imu" ,
            topic_imu,
            "sensor/imu" );

    std::string topic_output;
    ph.param< std::string >( "topic_output",
            topic_output,
            "localize/zeabus" );

    std::string topic_pressure;
    ph.param< std::string >( "topic_pressure", 
            topic_pressure,
            "sensor/pressure" );
    // Part setup for listen message form imu
    std::mutex lock_sensor_imu;
    sensor_msgs::Imu message_sensor_imu;
    zeabus_ros::subscriber::BaseClass< sensor_msgs::Imu > listener_sensor_imu( &nh, 
            &message_sensor_imu );
    listener_sensor_imu.setup_mutex_data( &lock_sensor_imu );
    listener_sensor_imu.setup_subscriber( topic_imu , 1 );

    // Part setup for listen message form pressure
    std::mutex lock_sensor_pressure;
    zeabus_utility::HeaderFloat64 message_sensor_pressure;
    zeabus_ros::subscriber::BaseClass< zeabus_utility::HeaderFloat64 > listener_sensor_pressure( &nh,
            &message_sensor_pressure );
    listener_sensor_pressure.setup_mutex_data( &lock_sensor_pressure );
    listener_sensor_pressure.setup_subscriber( topic_pressure , 1 );

    // Part setup publisher message for localize
    ros::Publisher publisher_localize = nh.advertise< nav_msgs::Odometry >( topic_output , 1 );

    // Part setup message for use in main loop
    nav_msgs::Odometry message_localize_state;
    zeabus_utility::HeaderFloat64 load_sensor_pressure;
    sensor_msgs::Imu load_sensor_imu;
    ros::Rate rate( frequency );

    while( ros::ok() )
    {
        // load message sensor imu
        lock_sensor_imu.lock();
        load_sensor_imu = message_sensor_imu;
        lock_sensor_imu.unlock();

        // load message sensor pressure
        lock_sensor_pressure.lock();
        load_sensor_pressure = message_sensor_pressure;
        lock_sensor_pressure.unlock();

        message_localize_state.pose.pose.position.z = load_sensor_pressure.data;

        // calculate velocity of robot
        calculate_angular_velocity( &load_sensor_imu , &message_localize_state.twist.twist.angular );
        // Get orientation of robot
        message_localize_state.pose.pose.orientation = load_sensor_imu.orientation;
        rate.sleep();
    } // main loop    
    
}
