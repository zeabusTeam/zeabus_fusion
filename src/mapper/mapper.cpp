// FILE			: mapper.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 07 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
#define SHOW_DATA // SHOW_DATA macro condition in calculate by force

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
    tf::Quaternion current_quaternion;
    double mass; 
    double moment_inertia;
    double volumn;

    boost::qvm::vec< double , 6 > vec_current_veclocity;
    vec_current_veclocity *= 0;
    boost::qvm::mat< double , 1 , 8 > mat_force_thruster;
    mat_force_thruster *= 0;

    std::mutex lock_message_current_force;
    zeabus_utility::Float64Array8 message_current_force;

    // Part setup parameter variable for use in ros system
    ph.param< std::string >( "file_map" , file_name , "qualification01.txt" );

    ph.param< std::string >( "topic_current_force" , topic_current_force , "/control/current_force");

    ph.param< double >( "moment_inertia" , moment_inertia , 10 ); 
    ph.param< double >( "volumn" , volumn , 0.038192 ); // unit meter square
    ph.param< double >( "mass" , mass , 35.05 ); // unit kilogram

    // Part varible initial setup and have to use data
    RobotForceHandle ch( mass , moment_inertia , volumn ); // calculate handle
    ch.setup_ptr_data( &current_quaternion ,
            &mat_force_thruster , 
            &vec_current_veclocity );

    zeabus_ros::subscriber::BaseClass< zeabus_utility::Float64Array8 > listener_current_force( &nh,
            &message_current_force);
    listener_current_force.setup_mutex_data( &lock_message_current_force );
    listener_current_force.setup_subscriber( topic_current_force , 1 );

    // Part setup ros variable to activate ros system
    fh.open( zeabus_ros::get_full_path( "zeabus_localize" , "parameter/map" , file_name ) );
    ( void )sth.setup( &fh );

    sth.spin( 10 );
    ros::Rate rate( 30 );

active_main:
    printf( "Start active main part\n");
    while( ros::ok() )
    {
        // Downlaod current force 
        lock_message_current_force.lock();
        boost::qvm::A00( mat_force_thruster ) = message_current_force.data[ 0 ];
        boost::qvm::A01( mat_force_thruster ) = message_current_force.data[ 1 ];
        boost::qvm::A02( mat_force_thruster ) = message_current_force.data[ 2 ];
        boost::qvm::A03( mat_force_thruster ) = message_current_force.data[ 3 ];
        boost::qvm::A04( mat_force_thruster ) = message_current_force.data[ 4 ];
        boost::qvm::A05( mat_force_thruster ) = message_current_force.data[ 5 ];
        boost::qvm::A06( mat_force_thruster ) = message_current_force.data[ 6 ];
        boost::qvm::A07( mat_force_thruster ) = message_current_force.data[ 7 ];
        lock_message_current_force.unlock();

        ch.calculate();
        rate.sleep();
    }

exit_main:
    return 0;
    
}
