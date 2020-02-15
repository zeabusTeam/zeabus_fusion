// FILE			: know_quaternion.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, October 29 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <tf/LinearMath/Quaternion.h>

#include    <iostream>

void print_quaternion( std::string message , tf::Quaternion quaternion )
{
    printf( "%s : %8.3f%8.3f%8.3f%8.3f\n", 
            message.c_str(),
            quaternion.x(), 
            quaternion.y(),
            quaternion.z(),
            quaternion.w() );
}

int main( int argv , char** argc )
{

    tf::Quaternion ned_quaternion( 0 , 0 , 0 , 1 );
    tf::Quaternion enu_quaternion( 0 , 0 , 0 , 1 );

    tf::Quaternion ned_to_enu_quaternion( 0.707 , 0.707 , 0 , 0 );
 
    print_quaternion( "NED TO QUATERNION",  
            ned_to_enu_quaternion*ned_quaternion );

    print_quaternion( "ENU TO QUATERNION",  
            ned_to_enu_quaternion*enu_quaternion );

    tf::Quaternion odom_to_robot;
    odom_to_robot.setRPY( 1.57 , 0 , -1.0 );
    tf::Quaternion vector( 10 , 0 , 0 , 0 );
    print_quaternion( "odom_to_robot" , odom_to_robot );
    print_quaternion( "inertia_to_body" , odom_to_robot * vector * odom_to_robot.inverse() );
    print_quaternion( "body_to_inertia" , odom_to_robot.inverse() * vector * odom_to_robot );
    print_quaternion( "test" , odom_to_robot * odom_to_robot.inverse() );
}
