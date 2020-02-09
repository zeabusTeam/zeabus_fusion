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

#include    <zeabus/ros/convert/geometry_quaternion.hpp>

#include    <zeabus_utility/Float64Array8.h>

#include    <nav_msgs/Odometry.h>

#include    <tf/LinearMath/Quaternion.h>

#include    <cstring> // include function memcpy

#include    <zeabus/escape_code.hpp>

void report( double* base_link_force , double* gravity_force , double* buoncy_force , 
        double* viscosity_force , double* summation_force , double* acceleration ,
        double* current_velocity );

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

    const double gravity = 9.78297;
    double mass;
    ph.param< double >( "weight" , mass , 35.05 ); // unit kilogram
    double weight = mass * gravity;

    double moment_inertia;
    ph.param< double >( "moment_inertia" , moment_inertia , 10 ); 

    const double rho_water = 1027.0;
    double volumn;
    ph.param< double >( "volumn" , volumn , 0.038192 ); // unit meter square
    double buoncy = rho_water * volumn * gravity;

    // Part setup subscribe in ros system
    // listen current force
    std::mutex lock_message_current_force;
    zeabus_utility::Float64Array8 message_current_force;
    zeabus_ros::subscriber::BaseClass< zeabus_utility::Float64Array8 > listener_current_force( &nh,
            &message_current_force);
    listener_current_force.setup_mutex_data( &lock_message_current_force );
    listener_current_force.setup_subscriber( topic_current_force , 1 );
    boost::array< double , 8 > current_force;
    
    // listen current state
    std::mutex lock_message_current_state;
    nav_msgs::Odometry message_current_state;
    zeabus_ros::subscriber::BaseClass< nav_msgs::Odometry > listener_current_state( &nh,
            &message_current_state );
    listener_current_state.setup_mutex_data( &lock_message_current_state );
    listener_current_state.setup_subscriber( topic_current_state , 1 );
    tf::Quaternion current_orientation( 0 , 0 , 0 , 1 ); 
    tf::Quaternion temp_quaternion( 0 , 0 , 0 , 1 );
   
    // message for output current map state

    boost::qvm::mat< double , 1 , 8 > thruster_force;
    boost::qvm::mat< double , 1 , 6 > base_link_force;
    boost::qvm::mat< double , 1 , 6 > viscosity_force;
    boost::qvm::mat< double , 1 , 6 > gravity_force;
    boost::qvm::mat< double , 1 , 6 > buoncy_force;
    boost::qvm::mat< double , 1 , 6 > summation_force;

    boost::qvm::vec< double , 3 > gravity_vector;
    boost::qvm::vec< double , 3 > buoncy_vector;
    boost::qvm::vec< double , 6 > constant_viscosity = { 20 , 30 , 40 , 30 , 40 , 25 }; 
    boost::qvm::vec< double , 6 > current_velocity = { 0 , 0 , 0 , 0 , 0 , 0 } ;
    boost::qvm::vec< double , 6 > previous_velocity = { 0 , 0 , 0 , 0 , 0 , 0 };
    boost::qvm::vec< double , 6 > acceleration;

    ros::Rate rate( frequency );

    ros::Time time_stamp; // use to know time now
    ros::Time previous_stamp; // use to save previous time to find now time
    ros::Time collect_stamp; // use to collect time of message current force
    double diff_time;

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
            previous_stamp = ros::Time::now();
            goto active_main;
        } // Mean you ever get new data
    }
active_main:
    std::cout   << "Now active mapping calculator\n";
    
    while( ros::ok() )
    {
        rate.sleep();
        time_stamp = ros::Time::now();
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
        base_link_force *= gravity;

        // load current orientation
        lock_message_current_state.lock();
        zeabus_ros::convert::geometry_quaternion::tf( &message_current_state.pose.pose.orientation,
                &current_orientation );
        lock_message_current_state.unlock();

        temp_quaternion = tf::Quaternion( 0 , 0 , buoncy , 0 ); // load buoncy to quaternion
        // transform buoncy from odom frame to robot frame
        temp_quaternion = current_orientation.inverse() * temp_quaternion * current_orientation;
        // load data buoncy force in robot frame
        buoncy_vector.a[0] = temp_quaternion.x();
        buoncy_vector.a[1] = temp_quaternion.y();
        buoncy_vector.a[2] = temp_quaternion.z();

        temp_quaternion = tf::Quaternion( 0 , 0 , -1.0*weight , 0 ); // load weight to quaternion
        // transform weight from odom frame to robot frame
        temp_quaternion = current_orientation.inverse() * temp_quaternion * current_orientation;
        // load data weight force in robot frame
        gravity_vector.a[0] = temp_quaternion.x();
        gravity_vector.a[1] = temp_quaternion.y();
        gravity_vector.a[2] = temp_quaternion.z();

        // prepare matrix of buoncy force
        std::memcpy( (void*) &buoncy_force.a[0][3] ,
                (void*)boost::qvm::cross( buoncy_vector , zeabus::robot::distance_center_buoncy ).a,
                sizeof( double ) * 3 ); 
        std::memcpy( (void*) buoncy_force.a[0] , (void*)buoncy_vector.a , sizeof( double )* 3 );

        // prepare matrix of gravity force
        std::memcpy( (void*) &gravity_force.a[0][3] , 
                (void*)boost::qvm::cross( gravity_vector, zeabus::robot::distance_center_gravity).a,
                sizeof( double ) * 3 );
        std::memcpy( (void*) gravity_force.a[0] , (void*)gravity_vector.a , sizeof( double )* 3 );

        // prepare matrix of viscosity force
        for( unsigned int run = 0 ; run < 6 ; run++ )
        {
            viscosity_force.a[0][ run ] = -1.0 *constant_viscosity.a[run] *previous_velocity.a[run];
        }

        // sum all force
        summation_force = viscosity_force + base_link_force + gravity_force + buoncy_force;

        // calculate acceleration of linear and angular
        acceleration.a[0] = summation_force.a[0][0] / mass;
        acceleration.a[1] = summation_force.a[0][1] / mass;
        acceleration.a[2] = summation_force.a[0][2] / mass;
        acceleration.a[3] = summation_force.a[0][3] / moment_inertia;
        acceleration.a[4] = summation_force.a[0][4] / moment_inertia;
        acceleration.a[5] = summation_force.a[0][5] / moment_inertia;
        diff_time = ( time_stamp - previous_stamp ).toSec();
        
        current_velocity = previous_velocity + ( acceleration * diff_time );

        // copy new velocity to previous for use next loop 
        std::memcpy( (void*) previous_velocity.a ,
                (void*) current_velocity.a ,
                sizeof( double ) * 6 );

        previous_stamp = time_stamp;


#ifdef PRINT_REPORTED
        report( base_link_force.a[0] , gravity_force.a[0] , buoncy_force.a[0] , 
                viscosity_force.a[0] , summation_force.a[0] , acceleration.a ,
                current_velocity.a );
#endif // PRINT_REPORTED
         
    }
}

void report( double* base_link_force , double* gravity_force , double* buoncy_force , 
        double* viscosity_force , double* summation_force , double* acceleration ,
        double* current_velocity )
{
//  printf( "%s", ( zeabus::escape_code::screen_clear + zeabus::escape_code::screen_home ).c_str());
    printf("Base Link  :         x|         y|         z|      roll|    pitch|       yaw\n" );
    printf("Thru Force :%10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\n" , 
            base_link_force[0] , base_link_force[1] , base_link_force[2] , 
            base_link_force[3] , base_link_force[4] , base_link_force[5] );
    printf("Grav Force :%10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\n" , 
            gravity_force[0] , gravity_force[1] , gravity_force[2] , 
            gravity_force[3] , gravity_force[4] , gravity_force[5] );
    printf("Buon Force :%10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\n" , 
            buoncy_force[0] , buoncy_force[1] , buoncy_force[2] , 
            buoncy_force[3] , buoncy_force[4] , buoncy_force[5] );
    printf("Visc Force :%10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\n" , 
            viscosity_force[0] , viscosity_force[1] , viscosity_force[2] , 
            viscosity_force[3] , viscosity_force[4] , viscosity_force[5] );
    printf("Sum Force  :%10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\n" , 
            summation_force[0] , summation_force[1] , summation_force[2] , 
            summation_force[3] , summation_force[4] , summation_force[5] );
    printf("Accel      :%10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\n" , 
            acceleration[0] , acceleration[1] , acceleration[2] , 
            acceleration[3] , acceleration[4] , acceleration[5] );
    printf("Current Vel:%10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\n" , 
            current_velocity[0] , current_velocity[1] , current_velocity[2] , 
            current_velocity[3] , current_velocity[4] , current_velocity[5] );
}
