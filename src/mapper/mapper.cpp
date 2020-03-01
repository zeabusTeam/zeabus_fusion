// FILE			: mapper.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 07 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
#define PRINT_RESULT

// MACRO CONDITION

#include    <mapper/mapper.hpp>

double bound_data( double value , double threshold );

int main( int argv , char** argc )
{
    zeabus_ros::Node node( argv , argc , "visual_map" );

    ros::NodeHandle ph( "~" );
    ros::NodeHandle nh( "" );

    node.spin();

    // Part variable inital setup <==========================================================
    zeabus::FileCSV fh; // file module to collect mapper
    zeabus_ros::StaticTF sth; // static transformation handle
    std::string file_name; // file to collect map detail <visual map>
    std::string topic_current_force; // Use to collect std::string of input topic current force
    std::string topic_current_state; // Use to collect std::string of input topic current state   
    tf::Quaternion current_quaternion( 0 , 0 , 0 , 1 );

    boost::qvm::vec< double , 6 > vec_current_velocity;
    vec_current_velocity *= 0;
    boost::qvm::mat< double , 1 , 8 > mat_force_thruster;
    mat_force_thruster *= 0;
    boost::qvm::vec< double , 6 > vec_previous_velocity;
    vec_previous_velocity *= 0;

    std::mutex lock_message_current_force;
    zeabus_utility::Float64Array8 message_current_force;

    std::mutex lock_message_current_state;
    nav_msgs::Odometry message_current_state;

    geometry_msgs::TransformStamped message_transform;
    message_transform.header.frame_id = "odom";
    message_transform.child_frame_id = "base_link_visual";
    message_transform.transform.rotation.w = 1;
    message_transform.transform.translation.x = 0;
    message_transform.transform.translation.y = 0;
    message_transform.transform.translation.z = 0;
    tf::TransformBroadcaster broadcaster;
    // Part setup parameter variable for use in ros system <=================================
    ph.param< std::string >( "file_map" , file_name , "activate.txt" );

    ph.param< std::string >( "topic_current_force" , topic_current_force , "/control/force/current");

    ph.param< std::string >( "topic_current_state" , topic_current_state , "/localize/zeabus");

    ph.param< double >( "moment_inertia" , moment_inertia , 10 ); 
    ph.param< double >( "volumn" , volumn , 0.038192 ); // unit meter square
    ph.param< double >( "mass" , mass , 35.05 ); // unit kilogram

    // Part varible initial setup and have to use data <=====================================
    RobotForceHandle ch; // calculate handle
    ch.setup_ptr_data( &current_quaternion ,
            &mat_force_thruster , 
            &vec_current_velocity );

    zeabus_ros::subscriber::BaseClass< zeabus_utility::Float64Array8 > listener_current_force( &nh,
            &message_current_force);
    listener_current_force.setup_mutex_data( &lock_message_current_force );
    listener_current_force.setup_subscriber( topic_current_force , 1 );

    zeabus_ros::subscriber::BaseClass< nav_msgs::Odometry > listener_current_state( &nh,
            &message_current_state );
    listener_current_state.setup_mutex_data( &lock_message_current_state );
    listener_current_state.setup_subscriber( topic_current_state , 1 );

    // Part setup ros variable to activate ros system <======================================
    fh.open( zeabus_ros::get_full_path( "zeabus_localize" , "parameter/map" , file_name ) );

    // Part setup dynamic roconfigure <======================================================
    dynamic_reconfigure::Server< zeabus_localize::VisualConfig > server_reconfigure;
    dynamic_reconfigure::Server< zeabus_localize::VisualConfig >::CallbackType
            function_reconfigure;
    function_reconfigure = boost::bind( &dynamic_reconfigure_callback , _1 , _2 );
    server_reconfigure.setCallback( function_reconfigure );
    zeabus_ros::DynamicReconfigure drh;
    drh.load( "zeabus_localize" , "parameter" , "visual.yaml" , ros::this_node::getName() );

    // Waiting first parameter for configure <===============================================
    bool updated = false;
    while( ros::ok() )
    {
        lock_dynamic_reconfigure.lock();
        if( avaliable_new_parameter )
        {
            avaliable_new_parameter = false;
            drh.dump( "zeabus_localize" , "parameter" , "visual.yaml" , ros::this_node::getName() );
            dynamic_reconfigure_set_parameter( &ch );
            updated = true;
        }
        lock_dynamic_reconfigure.unlock();
        if( updated ) break;
    }

    ( void )sth.setup( &fh );

    sth.spin( 10 );
    ros::Rate rate( 30 );
    ros::Time time_stamp = ros::Time::now(); // use to know time know
    ros::Time collect_stamp = time_stamp; // use to collect time of message current force
    ros::Time previous_stamp; // use to save previous time to find now time
    double diff_time = 0.0;
    tf::Quaternion temp_quaternion;

loop_first_data:
    printf( "Waiting for first data to start mission\n");
    while( ros::ok() )
    {
        rate.sleep();
        lock_message_current_force.lock();
        if( collect_stamp < message_current_force.header.stamp )
        {   
            collect_stamp = message_current_force.header.stamp;
        }   
        lock_message_current_force.unlock();
        if( collect_stamp != time_stamp )
        {   
            previous_stamp = ros::Time::now();
            goto active_main;
        } // Mean you ever get new data

    }

active_main:
    printf( "Start active main part\n");
    while( ros::ok() )
    {
        lock_dynamic_reconfigure.lock();
        if( avaliable_new_parameter )
        {
            dynamic_reconfigure_set_parameter( &ch );
            drh.dump( "zeabus_localize" , "parameter" , "visual.yaml" , ros::this_node::getName() );
            avaliable_new_parameter = false;
        }
        lock_dynamic_reconfigure.unlock();
        rate.sleep();
        time_stamp = ros::Time::now() ;
        // Downlaod current force 
        lock_message_current_force.lock();
        if( collect_stamp != message_current_force.header.stamp )
        {
            boost::qvm::A00( mat_force_thruster ) = message_current_force.data[ 0 ] *
                    zeabus::robot::gravity;
            boost::qvm::A01( mat_force_thruster ) = message_current_force.data[ 1 ] *
                    zeabus::robot::gravity;
            boost::qvm::A02( mat_force_thruster ) = message_current_force.data[ 2 ] *
                    zeabus::robot::gravity;
            boost::qvm::A03( mat_force_thruster ) = message_current_force.data[ 3 ] *
                    zeabus::robot::gravity;
            boost::qvm::A04( mat_force_thruster ) = message_current_force.data[ 4 ] *
                    zeabus::robot::gravity;
            boost::qvm::A05( mat_force_thruster ) = message_current_force.data[ 5 ] *
                    zeabus::robot::gravity;
            boost::qvm::A06( mat_force_thruster ) = message_current_force.data[ 6 ] *
                    zeabus::robot::gravity;
            boost::qvm::A07( mat_force_thruster ) = message_current_force.data[ 7 ] *
                    zeabus::robot::gravity;
            collect_stamp = message_current_force.header.stamp;
        }
        lock_message_current_force.unlock();
        // Download current quaternion
        lock_message_current_state.lock();
        zeabus_ros::convert::geometry_quaternion::tf( &message_current_state.pose.pose.orientation,
                &current_quaternion );
        message_transform.transform.rotation = message_current_state.pose.pose.orientation;
//        message_transform.transform.translation.z = message_current_state.pose.pose.position.z;
        lock_message_current_state.unlock();

        diff_time = ( time_stamp - previous_stamp ).toSec();
        ch.calculate( diff_time );
        // Now we have current data for use so I will find distance in robot frame
        temp_quaternion = tf::Quaternion(
            ( vec_current_velocity.a[ 0 ] + vec_previous_velocity.a[ 0 ] ) * diff_time / 2 ,
            ( vec_current_velocity.a[ 1 ] + vec_previous_velocity.a[ 1 ] ) * diff_time / 2 ,
            ( vec_current_velocity.a[ 2 ] + vec_previous_velocity.a[ 2 ] ) * diff_time / 2 ,
            0 );
#ifdef PRINT_RESULT
        printf( "DISTANCE ROBOT : %8.3f %8.3f %8.3f\n" , temp_quaternion.x() ,
                temp_quaternion.y() ,
                temp_quaternion.z() );
#endif // PRINT_RESULT
        temp_quaternion = current_quaternion * temp_quaternion * current_quaternion.inverse();
#ifdef PRINT_RESULT
        printf( "DISTANCE ODOM  : %8.3f %8.3f %8.3f\n" , temp_quaternion.x() ,
                temp_quaternion.y() ,
                temp_quaternion.z() );
#endif // PRINT_RESULT
        message_transform.header.stamp = ros::Time::now();
        message_transform.transform.translation.x += bound_data( temp_quaternion.x() , 0.005 );
        message_transform.transform.translation.y += bound_data( temp_quaternion.y() , 0.005 );
        message_transform.transform.translation.z += bound_data( temp_quaternion.z() , 0.005 );
#ifdef PRINT_RESULT
        printf( "THRESHOLD : 0.005\n" );
        printf( "DISTANCE ODOM  : %8.3f %8.3f %8.3f\n" , message_transform.transform.translation.x,
                message_transform.transform.translation.y,
                message_transform.transform.translation.z );
#endif
        broadcaster.sendTransform( message_transform );
        previous_stamp = time_stamp;
    }

exit_main:
    return 0;
    
}

double bound_data( double value , double threshold )
{
    double answer = 0;
    if( value > 0 )
    {
        if( value > threshold ) answer = value - threshold;
    } // case value is position
    else
    {
        if( value < -1.0*threshold ) answer = value + threshold;
    }
    return answer;
}
