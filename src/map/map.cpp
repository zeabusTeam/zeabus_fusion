// FILE			: map.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 01 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This programe responsible to connect between vision and localize system
//  We will receive data from camera and rotation to odom frame instanly focus what position in odom

// REFERENCE

// MACRO SET
#define _SHOW_SEQ_
#define _SHOW_CALCULATE_
#define _PUBLISHER_ODOMETRY_

// MACRO CONDITION
#ifdef _SHOW_CALCULATE_
    #undef _SHOW_SEQ_
#endif

#include    <map/map.hpp>

const tf::Quaternion rotation_to_optical_frame( -0.5 , 0.5 , -0.5 , 0.5 );

int main( int argv , char** argc )
{
    zeabus_ros::Node node( argv , argc , "map" );

    ros::NodeHandle ph( "~" );
    ros::NodeHandle nh( "" );

    node.spin();

    int result_program = 0;

    // SETUP PARAMETER FOR USE FROM ROS SYSTEM ====================================================
    std::string file_name;
    ph.param< std::string  >( "file_map" , file_name , "activate.txt" );

    int frequency;
    ph.param< int >( "frequency" , frequency , 30 );

    // SETUP VARIABLE AND OPERATOR IN ROS SYSTEM ==================================================
    // ======================================== PART STATIC TF DRAWE MAPPER
    zeabus::FileCSV fh; // File module to collect map detail
    zeabus_ros::StaticTF sth; // static transformation handle
    // below function load file
    fh.open( zeabus_ros::get_full_path( "zeabus_localize" , "parameter/map" , file_name ) ); 
    (void) sth.setup( &fh ); // setup static transformation
    sth.spin( frequency ); // start spin static transformation

    // ======================================== PART ON NORMAL ROS
    ros::Rate rate( frequency );
    unsigned int list_size = 2;
    std::string list_frame[2] = { "bottom_sign" , "front_sign" };
    ros::Time list_stamp[2] = { ros::Time::now() , ros::Time::now() };
    unsigned int list_seq[ 2 ] = { 0 , 0 };
#ifdef _PUBLISHER_ODOMETRY_
    geometry_msgs::Vector3Stamped message_odometry;
    ros::Publisher publish_odometry = nh.advertise< geometry_msgs::Vector3Stamped >( 
            "/localize/vision" , 1 );
#endif
    
    // ======================================== PART ON TF
    static tf::TransformListener listener_tf;
    tf::StampedTransform temp_transform;
    std::string temp_string;
    tf::Quaternion temp_quaternion;
    tf::Vector3 temp_vector3;
    tf::Quaternion rotation_bottom_camera; // rotation bottom camera to odom : vary
    // translation bottom camera to robot pose : constant
    tf::Vector3 translation_bottom_camera; 
    tf::Quaternion rotation_front_camera; // rotation front camera to odom : vary
    // translation front camera to robot pose : constant
    tf::Vector3 translation_front_camera;

    // ======================================== PART SUBSCRIBER SYSTEM
    std::mutex lock_message_vision_results;
    zeabus_utility::VisionResults message_vision_results;
    std::vector< zeabus_utility::VisionResult > vision_results;
    zeabus_ros::subscriber::BaseClass< zeabus_utility::VisionResults > listener_vision_results( &nh,
            &message_vision_results );
    listener_vision_results.setup_mutex_data( &lock_message_vision_results );
    listener_vision_results.setup_subscriber( "/vision/results" , 100 );

    // PART TEST SYSTEM ABOUT RESPONSE

listen_tf_activate:
    printf( "NODE MAP : Waiting for get translation to base_link\n" );
    while( ros::ok() )
    {
        try{
            listener_tf.lookupTransform( "bottom_camera",
                "base_link",
                ros::Time(0),
                temp_transform );
            translation_bottom_camera = temp_transform.getOrigin();
            printf( "Translation for bottom camera optical : %8.2f%8.2f%8.2f\n" , 
                    translation_bottom_camera.x(),
                    translation_bottom_camera.y(),
                    translation_bottom_camera.z() );

            listener_tf.lookupTransform( "front_camera",
                "base_link",
                ros::Time(0),
                temp_transform );
            translation_front_camera = temp_transform.getOrigin();
            printf( "Translation for front camera optical : %8.2f%8.2f%8.2f\n" , 
                    translation_front_camera.x(),
                    translation_front_camera.y(),
                    translation_front_camera.z() );
            printf( "Recive translation camera opitcal\n");
            goto active_main;
        }
        catch( tf::TransformException ex )
        {
            ROS_ERROR( "%s" , ex.what() );
            ros::Duration( 1.0 ).sleep();
        }
    }
 
active_main:
//    printf( "Start on time %f\n" , ros::Time::now().toSec() );
    while( ros::ok() )
    {
        rate.sleep();
        // Receive message
        lock_message_vision_results.lock();
        vision_results = message_vision_results.data;
        lock_message_vision_results.unlock();

        for( auto it = vision_results.begin() ; it != vision_results.end() ; it++ )
        {
            bool have_data = false;
            for( unsigned int run = 0 ; run < list_size ; run++ )
            {
                if( it->child_frame_id == list_frame[ run ] )
                {
                    if( it->stamp == list_stamp[ run ] )
                    {
                        ; // None to do anything 
                    }
                    else
                    {
                        if( it->stamp < list_stamp[ run ] )
                        {
                            list_seq[ run ] = 1;
                        }
                        else
                        {
                            list_seq[ run ]++;
                        }
                        list_stamp[ run ] = it->stamp;    
                        if( ! listener_tf.canTransform( "odom" , "base_link" , 
                                list_stamp[ run ] , &temp_string ) )
                        {
                            printf( "NODE MAP : %s\n" , temp_string.c_str() );
                            list_stamp[ run ] = ros::Time( 0 );
                        }
                        // Move vector translation to base_link frame
                        temp_vector3 = zeabus_ros::convert::geometry_vector3::tf( it->translation);
                        zeabus::math::inv_rotation( rotation_to_optical_frame , &temp_vector3 );
                        if( it->frame_id == "front_camera_optical" )
                        {
//                            temp_vector3 = ( translation_front_camera + 
//                                    zeabus_ros::convert::geometry_vector3::tf( it->translation ) );
                            listener_tf.lookupTransform( "odom" , "front_camera" , 
                                  list_stamp[ run ], temp_transform );
                            temp_vector3 += translation_front_camera;
                        }
                        else if( it->frame_id == "bottom_camera_optical" )
                        {
//                            temp_vector3 = ( translation_bottom_camera + 
//                                    zeabus_ros::convert::geometry_vector3::tf( it->translation ) );
                            listener_tf.lookupTransform( "odom" , "bottom_camera" , 
                                  list_stamp[ run ], temp_transform );
                            temp_vector3 += translation_bottom_camera;
                        }
                        else
                        {
                            printf( "NODE MAP : Fatal don\'t know frame %s\n" ,
                                    it->frame_id.c_str() );
                            break;
                        }
#ifdef _SHOW_CALCULATE_
                        printf( "%14s seq %5u receive :%10.3f%10.3f%10.3f\n" , 
                                it->child_frame_id.c_str() , list_seq[ run ],
                                it->translation.x, it->translation.y , it->translation.z );
                        printf( "%14s seq %5u robot   :%10.3f%10.3f%10.3f\n" , 
                                it->child_frame_id.c_str() , list_seq[ run ],
                                temp_vector3.x() , temp_vector3.y() ,temp_vector3.z() );
#endif
                        // Prepare data to coordinate of odom frame
//                        printf( "Request time %f\n" , list_stamp[ run ].toSec() );
//                        listener_tf.lookupTransform( "odom" , it->frame_id , 
//                              list_stamp[ run ], temp_transform );
                        zeabus::math::inv_rotation( temp_transform.getRotation() , &temp_vector3 );
#ifdef _SHOW_CALCULATE_
                        printf( "%14s seq %5u odom    :%10.3f%10.3f%10.3f\n" , 
                                it->child_frame_id.c_str() , list_seq[ run ],
                                temp_vector3.x() , temp_vector3.y() ,temp_vector3.z() );
#endif
                        listener_tf.lookupTransform( "odom" , it->child_frame_id , 
                                ros::Time(0), temp_transform );
                        temp_vector3 = temp_transform.getOrigin() - temp_vector3; 
#ifdef _SHOW_CALCULATE_
                        printf( "%14s seq %5u result  :%10.3f%10.3f%10.3f\n\n" , 
                                it->child_frame_id.c_str() , list_seq[ run ],
                                temp_vector3.x() , temp_vector3.y() ,temp_vector3.z() );
#endif
#ifdef _SHOW_SEQ
                        printf( "receive %14s from %14s seq %5u\n" , it->child_frame_id.c_str(),
                                it->frame_id.c_str() , 
                                list_seq[ run ] );
#endif
#ifdef _PUBLISHER_ODOMETRY_
                        message_odometry.header.stamp = ros::Time::now();
                        zeabus_ros::convert::geometry_vector3::tf( &temp_vector3 ,
                                &message_odometry.vector );
                        publish_odometry.publish( message_odometry );
#endif           
                    }
                    have_data = true;
                    break;
                }
                else
                {
                    ;
                }
            } // loop find frame 
            if( ! have_data )
            {
                printf( "NODE MAP : Fatal don\'t know frame %s\n" , it->child_frame_id.c_str() );
            }
        }
    } // lopp active   

exit_main:
    ros::shutdown();
    node.join();
    return result_program;
    
}
