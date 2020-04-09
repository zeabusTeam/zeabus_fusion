// FILE			: observer.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 09 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
//  ref01 : http://docs.ros.org/diamondback/api/tf/html/c++/classtf_1_1Transformer.html

// MACRO SET
// #define _PRINT_REUPDATE_POSITION_
// #define _AVERAGE_THRUSTER_FORCE_

// MACRO CONDITION

#include    <observer/observer.hpp>

nav_msgs::Odometry observer_data;
nav_msgs::Odometry localize_data;
nav_msgs::Odometry vision_data;
boost::qvm::mat< double , 6 , 1 > mat_force_thruster = { 0 , 0 , 0 , 0 , 0 , 0 };
#ifdef _AVERAGE_THRUSTER_FORCE_
boost::qvm::mat< double , 6 , 1 > mat_save_thruster = { 0 , 0 , 0 , 0 , 0 , 0 };
#endif
tf::Quaternion current_quaternion;

const double global_time_limit_buffer = 2; // unit second
std::vector< nav_msgs::Odometry > vec_observer_data;

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

    std::string topic_current_vision;
    ph.param< std::string >( "topic_current_vision" , 
            topic_current_vision,
            "/localize/vision" );

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
    // setup part listen current state
    std::mutex lock_message_current_state;
    nav_msgs::Odometry message_current_state;
    zeabus_ros::subscriber::BaseClass< nav_msgs::Odometry > listen_current_state( &nh,
            &message_current_state );
    listen_current_state.setup_mutex_data( &lock_message_current_state );
    listen_current_state.setup_subscriber( topic_current_state , 1 );
    // setup part listen vision data instead use tf
    std::mutex lock_message_current_vision;
    nav_msgs::Odometry message_current_vision;
    zeabus_ros::subscriber::BaseClass< nav_msgs::Odometry > listen_current_vision( &nh,
            &message_current_vision );
    listen_current_vision.setup_mutex_data( &lock_message_current_vision );
    listen_current_vision.setup_subscriber( topic_current_vision , 1 );

    // setup part tf we use tf for manage about transfer data
//    static tf::TransformListener tf_listener;
    static tf::TransformBroadcaster tf_broadcaster;
    ros::Time observer_stamp = ros::Time::now();
    ros::Time vision_stamp = observer_stamp;
    ros::Time localize_stamp = observer_stamp;
    ros::Time force_stamp = observer_stamp;
    tf::StampedTransform tf_transform;
    ros::Publisher publisher_observer = nh.advertise< nav_msgs::Odometry >( 
            "/localize/zeabus_observer" , 1 );

    // part variable use in main loop
    unsigned int mode = 0;
    double observer_diff = 0;
    double vision_diff = 0;
    double localize_diff = 0;
    observer_data.child_frame_id = "base_link_observer";
    observer_data.header.frame_id = "odom";
    ros::Rate rate( 30 );

    int tf_error = tf::NO_ERROR;
    std::string tf_error_string;

    reset_integral();

    load_parameter();
#ifdef _AVERAGE_THRUSTER_FORCE_
    mat_save_thruster *= 0;
#endif
    mat_force_thruster *= 0;
    while( ros::ok() )
    {
        rate.sleep();

        observer_stamp = ros::Time::now();
        // Download message current force of thruster
        lock_message_current_force.lock();
        if( force_stamp != message_current_force.header.stamp )
        {
            std::memcpy( (void*) mat_thruster_data.a ,
                    (void*) message_current_force.data.c_array(),
                    sizeof( double ) * 8 );
#ifndef _AVERAGE_THRUSTER_FORCE_
            mat_force_thruster = boost::qvm::transposed( mat_thruster_data * 
                    zeabus::robot::direction_all ) * zeabus::robot::gravity;
#else
            mat_save_thruster = boost::qvm::transposed( mat_thruster_data * 
                    zeabus::robot::direction_all ) * zeabus::robot::gravity;
            mat_force_thruster = ( mat_force_thruster + mat_save_thruster ) / 2; 
#endif
            force_stamp = message_current_force.header.stamp;
        }
        lock_message_current_force.unlock();

        // Download message current state
        lock_message_current_state.lock();
        if( localize_stamp != message_current_state.header.stamp )
        {
            localize_stamp = message_current_state.header.stamp;
            localize_data = message_current_state;
        }
        lock_message_current_state.unlock();

        // Download message current vision 
        lock_message_current_vision.lock();
        if( vision_stamp != message_current_vision.header.stamp )
        {
            vision_stamp = message_current_vision.header.stamp;
            vision_data = message_current_vision;
        }
        lock_message_current_vision.unlock();

response_vision_data:
        mode = vision_stamp_handle( vision_stamp );
        switch( mode )
        {
            case 2 :
                reset_vision(); 
            case 1 :
//  Collect for look problem about somthine don't have reason
//              tf_listener.lookupTransform( "base_link_vision" , "odom" , vision_stamp,
//                      tf_transform );
//              std::cout   << "At time " << vision_stamp << " Get data position is " 
//                          << vision_data.pose.pose.position.x << " " 
//                          << vision_data.pose.pose.position.y << "\n";
//                          << tf_transform.getOrigin().x() << " "
//                          << tf_transform.getOrigin().y() << "\n";
//              zeabus_ros::convert::nav_odometry::tf( &tf_transform , &vision_data );
                active_vision();
            default:
                break; 
        }

response_localize_data:
        mode = localize_stamp_handle( localize_stamp );
        switch( mode )
        {
            case 2 :
                reset_localize();
                reset_buffer_model();
                reset_buffer_observer();
            case 1 :
                active_localize();
            default:
                break;
        }
        // Finish receive data
        // current quaternion is quaternoin (orientation) of previous data
        zeabus_ros::convert::geometry_quaternion::tf( &observer_data.pose.pose.orientation,
                &current_quaternion );
        // go to part predict or tune parameter
        active_parameter();
        // after get parameter have to prepare data for calculate model
        //  Part linear velocity
        std::memcpy( (void*) arr_robot_velocity.c_array() ,
                (void*) arr_odom_linear_velocity.c_array() ,
                sizeof( double ) * 3 );
        zeabus::math::rotation( current_quaternion , arr_robot_velocity.c_array() );
        // Part angular velocity don't have to do becase we don't convert to odom frame
        // Now it time to calculate model for finding acceleration
        active_model( observer_stamp );
        // Now we have get acceleration we can calculate now
        observer_diff = ( observer_stamp - observer_data.header.stamp ).toSec();
        localize_diff = ( observer_stamp - localize_stamp ).toSec();
        vision_diff = ( observer_stamp - vision_stamp ).toSec(); 

        active_integral( observer_diff , localize_diff , vision_diff );
        // Last time after we have to do sequence next we publish data in ros system
        observer_data.header.stamp = observer_stamp;
        vec_observer_data.push_back( observer_data );
        check_buffer_observer( observer_stamp - ros::Duration( global_time_limit_buffer ) );
        zeabus_ros::convert::nav_odometry::tf( &observer_data , &tf_transform );
        tf_broadcaster.sendTransform( tf_transform );
        publisher_observer.publish( observer_data );
        // End loop 
    } // main loop

    dump_parameter();

exit_main:
    ros::shutdown();
    node.join();
    return 0;
}

void check_buffer_observer( const ros::Time& minimum_time )
{
    while( vec_observer_data.size() > 0 )
    {
        if( vec_observer_data.cbegin()->header.stamp < minimum_time )
        {
            vec_observer_data.erase( vec_observer_data.begin() );
        }
        else
        {
            break;
        }
    }
}

unsigned int vision_stamp_handle( const ros::Time stamp )
{
    static ros::Time save_stamp = stamp;
    unsigned int state = 0;
    if( save_stamp != stamp )
    {
        if( stamp < save_stamp )
        {
            std::cout   << zeabus::escape_code::bold_yellow << "MAIN OBSERVER : "
                        << zeabus::escape_code::normal_white 
                        << "reset time vision receive\n";
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
                        << "reset time localize receive\n";
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
    std::cout   << message <<  "\n";
}

void reset_buffer_observer()
{
    vec_observer_data.clear();
    observer_data.pose = localize_data.pose;
}
