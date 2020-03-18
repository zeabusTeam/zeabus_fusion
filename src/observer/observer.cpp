// FILE			: observer.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 09 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
//  ref01 : http://docs.ros.org/diamondback/api/tf/html/c++/classtf_1_1Transformer.html

// MACRO SET

// MACRO CONDITION

#include    <observer/observer.hpp>

nav_msgs::Odometry observer_data;
nav_msgs::Odometry localize_data;
nav_msgs::Odometry vision_data;
boost::qvm::mat< double , 6 , 1 > mat_force_thruster = { 0 , 0 , 0 , 0 , 0 , 0 };

int main( int argv, char** argc )
{
    zeabus_ros::Node node( argv , argc , "observer" );
    ros::NodeHandle nh( "" );
    ros::NodeHandle ph( "" );
    node.spin();

    // part parameter in ros systrem
    std::string topic_current_force;
    ph.param< std::string >( "topic_current_force" ,
            topic_current_force,
            "/control/force/current" );

    std::string topic_current_state;
    ph.param< std::string >( "topic_current_state" , 
            topic_current_state,
            "/localize/zeabus" );

    std::string frame_child_observer = "base_link_observer";
    std::string frame_parent = "odom";
    std::string frame_child_vision = "base_link_vision";
    
    // setup part listen current force
    std::mutex lock_message_current_force;
    zeabus_utility::Float64Array8 message_current_force;
    boost::qvm::mat< double , 1 , 8 > mat_thruster_data;
    zeabus_ros::subscriber::BaseClass< zeabus_utility::Float64Array8 > listen_current_force( &nh, 
            &message_current_force );
    listen_current_force.setup_mutex_data( &lock_message_current_force );
    listen_current_force.setup_subscriber( topic_current_force , 1 );

    std::mutex lock_message_current_state;
    nav_msgs::Odometry message_current_state;
    zeabus_ros::subscriber::BaseClass< nav_msgs::Odometry > listen_current_state( &nh,
            &message_current_state );
    listen_current_state.setup_mutex_data( &lock_message_current_state );
    listen_current_state.setup_subscriber( topic_current_state , 1 );

    // setup part tf we use tf for manage about transfer data
    static tf::TransformListener listener_tf;
    ros::Time vision_stamp = ros::Time::now();
    ros::Time localize_stamp = ros::Time::now();
    ros::Time force_stamp = ros::Time::now();
    ros::Time observer_stamp = ros::Time::now();
    tf::StampedTransform temp_transform;

    // part variable use in main loop
    unsigned int mode = 0;
    double observer_diff = 0;
    double vision_diff = 0;
    double localize_diff = 0;
    observer_data.child_frame_id = "base_link_observer";
    observer_data.header.frame_id = "odom";
    ros::Rate rate( 30 );

    while( ros::ok() )
    {
        rate.sleep();
        // Download message current force of thruster
        lock_message_current_force.lock();
        if( force_stamp != message_current_force.header.stamp )
        {
            std::memcpy( (void*) mat_thruster_data.a ,
                    (void*) message_current_force.data.c_array(),
                    sizeof( double ) * 8 );
            mat_force_thruster = boost::qvm::transposed( mat_thruster_data * 
                    zeabus::robot::direction_all ) * zeabus::robot::gravity;
            force_stamp = message_current_force.header.stamp;
        }
        lock_message_current_force.unlock();

        // Download message current state of thruster
        lock_message_current_state.lock();
        if( localize_stamp != message_current_state.header.stamp )
        {
            localize_stamp = message_current_state.header.stamp;
            localize_data = message_current_state;
        }
        lock_message_current_state.unlock();

        observer_stamp = ros::Time::now(); // observer stamp is stamp time of message observer

        mode = vision_stamp_handle( vision_stamp );
        switch( mode )
        {
            case 2 :
                reset_vision(); 
            case 1 :
                active_vision();
            default:
                break; 
        }
        mode = localize_stamp_handle( localize_stamp );
        switch( mode )
        {
            case 2 :
                reset_localize();
            case 1 :
                active_localize();
            default:
                break;
        }
        // Finish receive data
        // go to part predict or tune parameter
        active_parameter();
        // after get parameter have to calculate and get acceleration
        active_model();
        // Now we have get acceleration we can calculate now
        observer_diff = ( observer_stamp - observer_data.header.stamp ).toSec();
        localize_diff = ( observer_stamp - localize_stamp ).toSec();
        vision_diff = ( observer_stamp - vision_stamp ).toSec(); 
        active_integral( observer_diff , localize_diff , vision_diff );

        // Last time after we have to do sequence next we publish data in ros system
        observer_data.header.stamp = observer_stamp;
        // End loop 
    } // main loop

exit_main:
    ros::shutdown();
    node.join();
    return 0;
}

unsigned int vision_stamp_handle( const ros::Time stamp )
{
    static ros::Time save_stamp = ros::Time::now();
    unsigned int state = 0;
    if( save_stamp != stamp )
    {
        if( stamp < save_stamp )
        {
            std::cout   << zeabus::escape_code::bold_yellow << "MAIN OBSERVER : "
                        << zeabus::escape_code::normal_white 
                        << "Reverse time vision receive\n";
            state = 2;
        }
        else
        {
            state = 1;
        }
        save_stamp = stamp;
    }
    return state;
} // function vision_stamp_handle

unsigned int localize_stamp_handle( const ros::Time stamp )
{
    static ros::Time save_stamp = ros::Time::now();
    unsigned int state = 0;
    if( save_stamp != stamp )
    {
        if( stamp < save_stamp )
        {
            std::cout   << zeabus::escape_code::bold_yellow << "MAIN OBSERVER : "
                        << zeabus::escape_code::normal_white 
                        << "Reverse time localize receive\n";
            state = 2;
        }
        else
        {
            state = 1;
        }
        save_stamp = stamp;
    }
    return state;
} // function localize_stamp_handle

void publish( const std::string message )
{
    ;
}