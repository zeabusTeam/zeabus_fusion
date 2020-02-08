// FILE			: mapper.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 07 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <mapper.hpp>

int main( int argv , char** argc )
{
    zeabus_ros::Node node( argv , argc , "visual_map" );

    ros::NodeHandle ph( "~" );
    ros::NodeHandle nh( "" );

    node.spin();

    // Part variable inital setup
    zeabus::FileCSV fh; // file module to collect mapper
    zeabus_ros::StaticTF sth; // static transformation handle
    std::string file_name; // file to collect map detail <visual map>
    std::string topic_current_force; // Use to collect std::string of input topic current force
    std::string topic_current_state; // Use to collect std::string of input topic current state    

    ros::Rate rate( 30 );
    // Part setup parameter variable for use in ros system
    ph.param< std::string >( "file_map" , file_name , "qualification01.txt" );


    // Part setup ros variable to activate ros system
    fh.open( zeabus_ros::get_full_path( "zeabus_localize" , "parameter/map" , file_name ) );
    ( void )sth.setup( &fh );

    sth.spin( 10 );

active_main:
    while( ros::ok() )
    {
        rate.sleep();
    }

exit_main:
    return 0;
    
}
