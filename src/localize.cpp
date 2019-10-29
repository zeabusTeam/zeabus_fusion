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

#include    <geometry_msgs/Vector3.h>

#include    <zeabus_utility/HeaderFloat64.h>

#include    <zeabus/ros/node.hpp>

#include    <zeabus/ros/convert/geometry_quaternion.hpp>

#include    <zeabus/ros/subscriber/base_class.hpp>

#include    <tf/LinearMath/Quaternion.h>

#include    <tf/transform_broadcaster.h>

#include    <tf/transform_listener.h>

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
    static tf::TransformBroadcaster broadcaster;
    tf::Transform transform_localize_state;
    // set constant value
    message_localize_state.header.frame_id = "odom";
    message_localize_state.child_frame_id = "base_link";
    // Next step prepare about tramslation sensor handle
    static tf::TransformListener listener;
    tf::StampedTransform temp_transform;
    tf::Quaternion translation_pressure;
    tf::Quaternion rotation_imu;
    tf::Quaternion current_orientation;
    tf::Quaternion origin_orientation;
    try{
        // Part of pressure sensor
        listener.lookupTransform( "base_link" , "sensor_pressure" , ros::Time(0) , temp_transform );
        translation_pressure = tf::Quaternion( temp_transform.getOrigin().x(), 
                temp_transform.getOrigin().y(), 
                temp_transform.getOrigin().z(), 
                0 );
        // Part of imu sensor
        listener.lookupTransform( "base_link" , "sensor_imu_NED" , ros::Time(0) , temp_transform );
        rotation_imu = temp_transform.getRotation();

        geometry_msgs::Vector3 temp_vector3;
        tf::Matrix3x3( rotation_imu ).getRPY( temp_vector3.x , temp_vector3.y , temp_vector3.z );
        printf( "---------------------- TRANSLATION OF LOCALIZE --------------------------\n");
        printf( "rotation for IMU (YPR): %6.2f%6.3f%6.2f", 
                temp_vector3.z, 
                temp_vector3.y , 
                temp_vector3.x );
        printf( "translation for depth : %6.2f%6.2f%6.2f",
                translation_pressure.x(),
                translation_pressure.y(),
                translation_pressure.z() );

    }
    catch( tf::TransformException ex )
    {
        ROS_ERROR( "%s" , ex.what() );
        goto exit_main;
    }

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

        zeabus_ros::convert::geometry_quaternion::tf( &load_sensor_imu.orientation, 
                &origin_orientation );
        current_orientation = rotation_imu * origin_orientation * rotation_imu.inverse();

        // calculate velocity of robot
        calculate_angular_velocity( &current_orientation, 
                &message_localize_state.twist.twist.angular,
                load_sensor_imu.header.stamp );
        // Get orientation of robot
        message_localize_state.pose.pose.orientation = load_sensor_imu.orientation;
        message_localize_state.header.stamp = ros::Time::now();

        // Rotation value of pressure sensor
        message_localize_state.pose.pose.position.z = load_sensor_pressure.data +
                ( current_orientation * translation_pressure * current_orientation.inverse() ).z();
                 
        // Set variable transform 
        transform_localize_state.setOrigin( tf::Vector3( 
                message_localize_state.pose.pose.position.x,
                message_localize_state.pose.pose.position.y, 
                message_localize_state.pose.pose.position.z ) );
        transform_localize_state.setRotation( tf::Quaternion( 
                message_localize_state.pose.pose.orientation.x,
                message_localize_state.pose.pose.orientation.y,
                message_localize_state.pose.pose.orientation.z,
                message_localize_state.pose.pose.orientation.w ) );
        broadcaster.sendTransform( tf::StampedTransform( transform_localize_state,
                message_localize_state.header.stamp,
                message_localize_state.header.frame_id,
                message_localize_state.child_frame_id ) );
        rate.sleep();
    } // main loop    

exit_main:
    ros::shutdown();
    
}
