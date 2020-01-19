// FILE			: know_force.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, January 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
#define PRINT_REPORTED

// MACRO CONDITION

#include    <zeabus/robot.hpp>

#include    <ros/ros.h>

#include    <boost/qvm/vec_access.hpp>

#include    <boost/qvm/mat_access.hpp>

#include    <iostream>

#include    <zeabus/ros/node.hpp>

#include    <zeabus/ros/subscriber/base_class.hpp>

#include    <zeabus_utility/Float64Array8.h>

void report( double* base_link_force );

int main( int argv , char** argc )
{

    zeabus_ros::Node node( argv , argc , "listen_force" );

    ros::NodeHandle ph("~"); // param handle
    ros::NodeHandle nh(""); // node handle

    node.spin();

    // Part setup parameter variable for use in ros system
    int frequency;
    ph.param< int >( "frequency" , frequency , 30 );

    std::string topic_current_force;
    ph.param< std::string >( "topic_input_force" , topic_current_force , "control/force/current" );

    std::string topic_current_state;
    ph.param< std::string >( "topic_current_state" , topic_current_state , "localize/zeabus" );


    // Part setup subscribe in ros system
    std::mutex lock_message_current_force;
    zeabus_utility::Float64Array8 message_current_force;
    zeabus_ros::subscriber::BaseClass< zeabus_utility::Float64Array8 > listener_current_force( &nh,
            &message_current_force);
    listener_current_force.setup_mutex_data( &lock_message_current_force );
    listener_current_force.setup_subscriber( topic_current_force , 1 );
    boost::array< double , 8 > current_force;

    boost::qvm::mat< double , 1 , 8 > thruster_force;
    boost::qvm::mat< double , 1 , 6 > base_link_force;;

    ros::Rate rate( frequency );

    ros::Time time_stamp; // use to know time now
    ros::Time collect_stamp; // use to collect time of message current force

loop_first_data:

    time_stamp = ros::Time::now();
    collect_stamp = time_stamp;
    std::cout   << "Waiting for first data to start mission\n";

    while( ros::ok() )
    {
        rate.sleep();
        lock_message_current_force.lock();
        if( collect_stamp < message_current_force.header.stamp )
        {
            current_force = message_current_force.data;
            collect_stamp = message_current_force.header.stamp;
        }
        lock_message_current_force.unlock();
        if( collect_stamp != time_stamp )
        {
            for( unsigned int run = 0 ; run < 8 ; run++ )
            {
                thruster_force.a[ 0 ][ run ] = current_force[ run ];
            }
            time_stamp = ros::Time::now();
            goto active_main;
        } // Mean you ever get new data
    }
active_main:
    std::cout   << "Now active mapping calculator\n";
    
    while( ros::ok() )
    {
        rate.sleep();
        // load data of current force
        lock_message_current_force.lock();
        if( collect_stamp != message_current_force.header.stamp )
        {
            current_force = message_current_force.data;
            collect_stamp = message_current_force.header.stamp;
        }
        lock_message_current_force.unlock();
        // load boost array to boost qvm mat
        for( unsigned int run = 0 ; run < 8 ; run++ )
        {
            thruster_force.a[ 0 ][ run ] = current_force[ run ];
        }
        // calculate base link force
        base_link_force = thruster_force * zeabus::robot::direction_all;

#ifdef PRINT_REPORTED
        report( base_link_force.a[0] );
#endif // PRINT_REPORTED
         
    }
}

void report( double* base_link_force )
{
    printf("           :         x|         y|         z|      roll|    pitch|       yaw\n" );
    printf("Base Force :%10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\n" , 
            base_link_force[0] , base_link_force[1] , base_link_force[2] , 
            base_link_force[3] , base_link_force[4] , base_link_force[5] );
}
