// FILE			: localize.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, October 24 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
// _ROTATION_IMU_SYSTEM_    : If define this mean you want to conversation between ENU<-->NED

// README

// REFERENCE

// MACRO SET
#define _ROTATION_IMU_SYSTEM_

// MACRO CONDITION

#include    <mutex>

#include    <iostream>

#include    <sensor_msgs/Imu.h>

#include    <nav_msgs/Odometry.h>

#include    <geometry_msgs/Pose.h>

#include    <geometry_msgs/Vector3.h>

#include    <zeabus_utility/HeaderFloat64.h>

#include    <zeabus/ros/node.hpp>

#include    <zeabus/ros/convert/geometry_quaternion.hpp>

#include    <zeabus/ros/subscriber/base_class.hpp>

#include    <tf/LinearMath/Quaternion.h>

#include    <tf/transform_broadcaster.h>

#include    <tf/transform_listener.h>

#include    <localize/target_service.hpp>

#include    <localize/state_service.hpp>

#include    <localize/calculate_angular_velocity.hpp>

#include    <localize/calculate_linear_velocity.hpp>

#include    <localize/updated_target_state.hpp>

const double timeout = 0.2;

int main( int argv , char** argc )
{
    zeabus_ros::Node node( argv , argc , "localize" );

    ros::NodeHandle ph("~"); // for param node handle
    ros::NodeHandle nh(""); // general node handle

    AccelerationHandle ah; // acceleration handle
    ah.reset_state();

    node.spin();
    // This frequency is tell you about frequency to calculate new state of robot
    int frequency;
    ph.param< int >( "frequency" , frequency , 30 );    

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
    message_sensor_imu.orientation.x = 0;
    message_sensor_imu.orientation.y = 0;
    message_sensor_imu.orientation.z = 0;
    message_sensor_imu.orientation.w = 1;
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
    double buffer_velocity[5] = { 0 , 0 , 0 , 0 , 0 };
    unsigned int point_buffer_velocity = 0;
    double sum_velocity = 0;

    // Part setup publisher message for localize
    ros::Publisher publisher_localize = nh.advertise< nav_msgs::Odometry >( topic_output , 1 );

    // Part setup message for use in main loop
    ros::Time time_stamp;
    std::mutex lock_localize_state;
    nav_msgs::Odometry message_localize_state;
    message_localize_state.pose.pose.orientation.w = 1;
    zeabus_utility::HeaderFloat64 load_sensor_pressure;
    sensor_msgs::Imu load_sensor_imu;
    ros::Rate rate( frequency );
    static tf::TransformBroadcaster broadcaster;
    tf::Transform transform_localize_state;
    // set constant value
    message_localize_state.header.frame_id = "odom";
    message_localize_state.child_frame_id = "base_link";

    // Part about set target state
    std::mutex lock_target_state;
    geometry_msgs::Pose message_target_state;
    tf::Transform transform_target_state;
    message_target_state.position.x = 0;
    message_target_state.position.y = 0;
    message_target_state.position.z = 0;
    message_target_state.orientation.x = 0;
    message_target_state.orientation.y = 0;
    message_target_state.orientation.z = 0;
    message_target_state.orientation.w = 1;
    bool update_target_state = true;

    // Part manage all service for ros to set target state
    TargetService target_service( &nh );
    target_service.setup_all_variable( &update_target_state,
            &message_target_state,
            &lock_target_state,
            &( message_localize_state.pose.pose ),
            &lock_localize_state );
    target_service.setup_all_service();

    // Part manage all service for set current position
    StateService state_service( &nh );
    state_service.setup_all_variable( &message_localize_state,
            &lock_localize_state );
    state_service.setup_all_service();

    // Part about updated target state by control type subscriber
    UpdatedTargetState utsh( &nh ); // Updated target state handle
    utsh.setup_all_variable( &update_target_state, 
            &message_target_state, 
            &lock_target_state,
            &( message_localize_state.pose.pose ),
            &lock_localize_state );
    utsh.setup_subscriber( "localize/reset" , timeout );

    // Next step prepare about tramslation sensor handle
    static tf::TransformListener listener;
    tf::StampedTransform temp_transform;
    tf::Quaternion translation_pressure;
    tf::Quaternion rotation_imu;
    tf::Quaternion current_orientation( 0 , 0 , 0 , 1);
    tf::Quaternion origin_orientation ( 0 , 0 , 0 , 1);
#ifdef _ROTATION_IMU_SYSTEM_
    const static tf::Quaternion conversation_coordinate( 0.707 , 0.707 , 0 , 0 );
#endif // #ifdef _ROTATION_IMU_SYSTEM_
    while( ros::ok() )
    {
        try{
            // Part of pressure sensor
            printf( "---------------------- TRANSLATION OF LOCALIZE --------------------------\n");
            listener.lookupTransform( "base_link", 
                    "sensor_pressure", 
                    ros::Time(0), 
                    temp_transform );
            translation_pressure = tf::Quaternion( temp_transform.getOrigin().x(), 
                    temp_transform.getOrigin().y(), 
                    temp_transform.getOrigin().z(), 
                    0 );
            printf( "translation for depth : %8.2f%8.2f%8.2f\n",
                    translation_pressure.x(),
                    translation_pressure.y(),
                    translation_pressure.z() );

            // Part of imu sensor about 
            listener.lookupTransform( "base_link",
                    "sensor_imu",
                    ros::Time(0),
                    temp_transform );
            rotation_imu = temp_transform.getRotation();
            geometry_msgs::Vector3 temp_vector3;
            tf::Matrix3x3( rotation_imu ).getRPY( temp_vector3.x , temp_vector3.y , temp_vector3.z );
            printf( "rotation for IMU (YPR): %8.2f%8.3f%8.2f\n", 
                    temp_vector3.z, 
                    temp_vector3.y , 
                    temp_vector3.x );

            listener.lookupTransform( "base_link",
                    "sensor_imu_NED",
                    ros::Time( 0 ),
                    temp_transform );
            tf::Matrix3x3( temp_transform.getRotation() ).getRPY( temp_vector3.x, 
                    temp_vector3.y, 
                    temp_vector3.z );
            ah.setup_rotation_quaternion( temp_transform.getRotation() );
            printf( "rotation for acceleration (YPR): %8.2f%8.3f%8.2f\n", 
                    temp_vector3.z, 
                    temp_vector3.y , 
                    temp_vector3.x );

            goto active_main;
        }
        catch( tf::TransformException ex )
        {
            ROS_ERROR( "%s" , ex.what() );
            ros::Duration( 1.0 ).sleep();
        }
    }

active_main:
    while( ros::ok() )
    {
        // Critical section to access about target state of robot
        lock_target_state.lock();
        if( update_target_state )
        {
            transform_target_state.setOrigin( tf::Vector3( message_target_state.position.x,
                    message_target_state.position.y,
                    message_target_state.position.z ) );
            transform_target_state.setRotation( tf::Quaternion( 
                    message_target_state.orientation.x,
                    message_target_state.orientation.y,
                    message_target_state.orientation.z,
                    message_target_state.orientation.w ) );
            update_target_state = false;
        }

        lock_target_state.unlock();
        // load message sensor imu
        lock_sensor_imu.lock();
        load_sensor_imu = message_sensor_imu;
        lock_sensor_imu.unlock();

        // load message sensor pressure
        lock_sensor_pressure.lock();
        load_sensor_pressure = message_sensor_pressure;
        lock_sensor_pressure.unlock();

        lock_localize_state.lock();
        message_localize_state.pose.pose.position.z = load_sensor_pressure.data;
        lock_localize_state.unlock();

        zeabus_ros::convert::geometry_quaternion::tf( &load_sensor_imu.orientation, 
                &origin_orientation );

#ifdef _ROTATION_IMU_SYSTEM_
        origin_orientation = conversation_coordinate.inverse() * 
                origin_orientation * 
                conversation_coordinate;
#endif // _ROTATION_IMU_SYSTEM_

        current_orientation = rotation_imu * origin_orientation;
        current_orientation = tf::Quaternion( -1.0*current_orientation.x() ,
                -1.0*current_orientation.y(),
                current_orientation.z(),
                current_orientation.w() );

        // calculate velocity of robot
        calculate_angular_velocity( &current_orientation, 
                &message_localize_state.twist.twist.angular,
                load_sensor_imu.header.stamp );
        // Get orientation of robot
        lock_localize_state.lock();
        zeabus_ros::convert::geometry_quaternion::tf( &current_orientation, 
                &message_localize_state.pose.pose.orientation ) ;
        message_localize_state.header.stamp = ros::Time::now();
        // Rotation value of pressure sensor 
        // Pressure sensor is base_link convert to odom frame
        double previous_data = message_localize_state.pose.pose.position.z;
        message_localize_state.pose.pose.position.z = load_sensor_pressure.data -
                ( current_orientation * translation_pressure * current_orientation.inverse() ).z();
        lock_localize_state.unlock();

        sum_velocity -= buffer_velocity[ point_buffer_velocity ]; 
        buffer_velocity[ point_buffer_velocity ] = ( message_localize_state.pose.pose.position.z - 
                previous_data ) * frequency; 
        sum_velocity += buffer_velocity[ point_buffer_velocity ];
        point_buffer_velocity = ( point_buffer_velocity  + 1) / 5;
        message_localize_state.twist.twist.linear.z = sum_velocity / 5 ;
//      message_localize_state.twist.twist.linear.z = ( message_localize_state.pose.pose.position.z- 
//                previous_data ) * frequency; 

//        ah.updated( &load_sensor_imu.header.stamp , &load_sensor_imu.linear_acceleration );
//        ah.get_velocity( &message_localize_state.twist.twist.linear );

        utsh.updated( &message_localize_state.header.stamp );

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
        broadcaster.sendTransform( tf::StampedTransform( transform_target_state,
                message_localize_state.header.stamp,
                message_localize_state.header.frame_id,
                message_localize_state.child_frame_id + "_target" ) );

        publisher_localize.publish( message_localize_state );
        time_stamp = message_localize_state.header.stamp;
        rate.sleep();
    } // main loop    

exit_main:
    ros::shutdown();
    
}
