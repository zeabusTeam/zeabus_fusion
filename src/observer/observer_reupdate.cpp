// FILE			: observer_reupdate.cpp
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

ros::Publisher publisher_point;

bool reupdate_velocity( const nav_msgs::Odometry& vision_data )
{
    zeabus_utility::ObserverPoint message_point;
    bool have_reupdate = false;
    // Now we have velocity at time and have position so you can abort data in vec_observer_data
    //  But before abort you must to ensure that data 
    for( auto it = vec_observer_data.begin(); it != vec_observer_data.end() ; it++ )
    {
        if( it->header.stamp > vision_data.header.stamp )
        {
            have_reupdate = true;
            // part publish data for echo
            message_point.vision_point = vision_data.pose.pose.position;
            message_point.observer_point = it->pose.pose.position;
            message_point.vision_stamp = vision_data.header.stamp;
            message_point.observer_stamp = it->header.stamp;
            message_point.error_point.x =  
                    ( message_point.observer_point.x - message_point.vision_point.x );
            message_point.error_point.y = 
                    ( message_point.observer_point.y - message_point.vision_point.y );
            message_point.error_point.z = 
                    ( message_point.observer_point.z - message_point.vision_point.z );
            publisher_point.publish( message_point );
            // end part publish data for echo

        }
    }
} // function reupdate_velocity

bool reupdate_model( const nav_msgs::Odometry& vision_data )
{
    
} // function reupdate_model

bool reupdate_position( const nav_msgs::Odometry& vision_data )
{
//    static ros::Publisher publisher_point = ros::NodeHandle("").advertise< 
//            zeabus_utility::ObserverPoint >( "/localize/observer/point" , 1 );
    static zeabus_utility::ObserverPoint message_point;
    bool have_reupdate = false;
#ifdef _PRINT_REUPDATE_POSITION_
    unsigned int count = 0;
#endif // _PRINT_REUPDATE_POSITION_
    for( auto it = vec_observer_data.begin(); it != vec_observer_data.end() ; it++ )
    {
        // I have start at index 1 because we have to use two data for integral
        if( it->header.stamp > vision_data.header.stamp )
        {
            have_reupdate = true;
            message_point.vision_point = vision_data.pose.pose.position;
            message_point.observer_point = it->pose.pose.position;
            message_point.vision_stamp = vision_data.header.stamp;
            message_point.observer_stamp = it->header.stamp;
            message_point.error_point.x =  
                    ( message_point.observer_point.x - message_point.vision_point.x );
            message_point.error_point.y = 
                    ( message_point.observer_point.y - message_point.vision_point.y );
            message_point.error_point.z = 
                    ( message_point.observer_point.z - message_point.vision_point.z );
            publisher_point.publish( message_point );
            if( it == vec_observer_data.begin() ) break;
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

#ifdef _PRINT_REUPDATE_POSITION_
            printf( "Match on order %4u from %4lu Observer %8.3f:%8.3f Vision %8.3f:%8.3f\n",
                    count , vec_observer_data.size(),
                    it->pose.pose.position.x , it->pose.pose.position.y,
                    vision_data.pose.pose.position.x , vision_data.pose.pose.position.y );
#endif // _PRINT_REUPDATE_POSITION_

            for( auto sub_it = it ; sub_it != vec_observer_data.end() ; sub_it++ )
            {
                sub_it->pose.pose.position.x -= diff_x;
                sub_it->pose.pose.position.y -= diff_y;
            }
            observer_data.pose.pose.position.x -= diff_x;
            observer_data.pose.pose.position.y -= diff_y;
            // can deletate because past data don't important to collect them
            // Delete will help you to filter time search
            // But this vector have time only 2 second? I desire that.
            // Don't worry time to access data
            vec_observer_data.erase( vec_observer_data.begin() , it );
            break; // after update mean you run overdata
        } // case (it--)->header.stamp < vision_data.header.stamp < it->header.stamp
        else
        {
#ifdef _PRINT_REUPDATE_POSITION_
            count++;
#else
            ;
#endif
        } 
    }
    if( ! have_reupdate && vec_observer_data.size() > 1 )
    {
        std::cout   << "MAIN OBSERVER : vision data is old over limit period\n";
    }
    else if( ! have_reupdate && vec_observer_data.size() == 0 )
    {
        std::cout   << "MAIN OBSERVER : don't have data in buffer for updated\n";
    }
    else
    {
        ;
    }
    return have_reupdate;
} // return true in case have to reupdate data and false in case that is last data in buffer

void setup_reupdate()
{
    publisher_point = ros::NodeHandle("").advertise< 
            zeabus_utility::ObserverPoint >( "/localize/observer/point" , 1 );
    return;
}
