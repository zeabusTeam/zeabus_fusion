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

#include    <tf/LinearMath/Quaternion.h>

#include    <zeabus/ros/node.hpp>

#include    <zeabus/ros/subscriber/base_class.hpp>

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
    // Part setup for listen message for imu
    std::mutex lock_sensor_imu;
    sensor_msgs::Imu message_sensor_imu;
    zeabus_ros::subscriber::BaseClass< sensor_msgs::Imu > listener_sensor_imu( &nh, 
            &message_sensor_imu);
    listener_sensor_imu.setup_mutex_data( &lock_sensor_imu );
    listener_sensor_imu.setup_subscriber( topic_imu , 1 );

    // Part setup publisher message for localize
    ros::Publisher publisher_localize = nh.advertise< nav_msgs::Odometry >( topic_output , 1 );

    // Part setup message for use in main loop
    nav_msgs::Odometry message_localize_state;
    sensor_msgs::Imu load_sensor_imu;
    ros::Rate rate( frequency );

    while( ros::ok() )
    {
        lock_sensor_imu.lock();
        load_sensor_imu = message_sensor_imu;
        lock_sensor_imu.unlock();
        // calculate velocity of robot
        calculate_angular_velocity( &load_sensor_imu , &message_localize_state.twist.twist.angular );
        // Get orientation of robot
        message_localize_state.pose.pose.orientation = load_sensor_imu.orientation;
        rate.sleep();
    } // main loop    
    
}
