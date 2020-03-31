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
    static tf::TransformListener tf_listener;
    static tf::TransformBroadcaster tf_broadcaster;
    ros::Time vision_stamp = ros::Time::now();
    ros::Time localize_stamp = ros::Time::now();
    ros::Time force_stamp = ros::Time::now();
    ros::Time observer_stamp = ros::Time::now();
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

    while( ros::ok() )
    {
        rate.sleep();

        observer_stamp = ros::Time::now(); // observer stamp is stamp time of message observer

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

        // Get vision stamp by look tf data because vision data only send when found
        tf_error = tf_listener.getLatestCommonTime( "odom" ,
                    "base_link_vision",
                    vision_stamp,
                    &tf_error_string );
        if( tf_error != tf::NO_ERROR )
        {
            std::cout   << "MAIN OBSERVER : error listen vision data\n";
            goto response_localize_data;
        }
        else
        {
            tf_listener.lookupTransform( "base_link_vision" , "odom" , vision_stamp,
                    tf_transform );
            zeabus_ros::convert::nav_odometry::tf( &tf_transform , &vision_data );
        }

response_vision_data:
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

response_localize_data:
        mode = localize_stamp_handle( localize_stamp );
        switch( mode )
        {
            case 2 :
                reset_localize();
                reset_buffer_model();
                vec_observer_data.clear();
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
        zeabus_ros::convert::nav_odometry::tf( &observer_data , &tf_transform );
        tf_broadcaster.sendTransform( tf_transform );
        // End loop 
    } // main loop

exit_main:
    ros::shutdown();
    node.join();
    return 0;
}

bool reupdate_position( const nav_msgs::Odometry& vision_data )
{
    bool have_reupdate = false;
    for( auto it = vec_observer_data.begin() + 1 ; it != vec_observer_data.end() ; it++ )
    {
        // I have start at index 1 because we have to use two data for integral
        if( it->header.stamp > vision_data.header.stamp )
        {
            have_reupdate = true;
            // start part reupdate data instanly
            double diff_origin = ( it->header.stamp - ( it - 1 )->header.stamp ).toSec();
            double diff_newer = ( ( it->header.stamp ) - vision_data.header.stamp ).toSec();
            double time_ratio = diff_newer / diff_origin;
            // equation d_p = x_2_o - x_2_n
            //          d_p = x_2_o - ( x_1_n + ( x_2_o - x_1_o ) / d_t_o * d_t_n )
            double diff_x = it->pose.pose.position.x - 
                    vision_data.pose.pose.position.x -
                    (it->pose.pose.position.x - (it-1)->pose.pose.position.x ) * time_ratio; 

            double diff_y = it->pose.pose.position.y - 
                    vision_data.pose.pose.position.y -
                    (it->pose.pose.position.y - (it-1)->pose.pose.position.y ) * time_ratio;

            for( auto sub_it = it ; sub_it != vec_observer_data.end() ; sub_it++ )
            {
                sub_it->pose.pose.position.x -= diff_x;
                sub_it->pose.pose.position.y -= diff_y;
            }
            // can deletate because past data don't important to collect them
            // Delete will help you to filter time search
            // But this vector have time only 2 second? I desire that.
            // Don't worry time to access data
            vec_observer_data.erase( vec_observer_data.begin() , it );
            break; // after update mean you run overdata
        } // case (it--)->header.stamp < vision_data.header.stamp < it->header.stamp
        else
        {
            ;
        } 
    }
    if( ! have_reupdate && vec_observer_data.size() > 1 )
    {
        vec_observer_data.rbegin()->pose.pose.position.x = vision_data.pose.pose.position.x;
        vec_observer_data.rbegin()->pose.pose.position.y = vision_data.pose.pose.position.y;
    }
    return have_reupdate;
} // return true in case have to reupdate data and false in case that is last data in buffer

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
    static ros::Time save_stamp = ros::Time::now();
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
    ;
}
