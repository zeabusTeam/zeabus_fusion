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
#define _PUBLISH_COMPARE_POINT_

// MACRO CONDITION

#include    <observer/observer.hpp>

ros::Publisher publisher_point;

bool reupdate_calculate( const nav_msgs::Odometry& vision_data , const tf::Vector3& vec_velocity )
{
    bool have_reupdate = false;
    // Now we have velocity at time and have position so you can abort data in vec_observer_data
    //  But before abort you must to ensure that data 
#ifdef _PUBLISH_COMPARE_POINT_
    zeabus_utility::ObserverPoint message_point;
    for( auto it = vec_observer_data.cbegin(); it != vec_observer_data.cend() ; it++ )
    {
        if( it->header.stamp >= vision_data.header.stamp )
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
#endif // _PUBLISH_COMPARE_POINT_
    double velocity[3] = { vec_velocity.x(), 
            vec_velocity.y(), vec_velocity.z() }; // velocity odom frame
    double acceleration[ 3 ] = { 0 , 0 , 0 };
    double position[ 2 ] = { vision_data.pose.pose.position.x ,
            vision_data.pose.pose.position.y };
    auto cit = vec_model_data.cbegin(); // constant iteration
    auto oit = vec_observer_data.begin();  // observer iteration
    auto stamp = vision_data.header.stamp;
    while( cit != vec_model_data.cend() )
    {
        if( cit->stamp >= vision_data.header.stamp ) // condition it time to recalculate
        {
            have_reupdate = true; // first time have update data
            if( cit->stamp != oit->header.stamp )
            {
                std::cout   << zeabus::escape_code::bold_yellow << "WARNING " 
                            << zeabus::escape_code::normal_white 
                            << "REUPDATED OBSERVER : observer<->model time did't match\n";
                while( oit != vec_observer_data.end() )
                {
                    if( oit->header.stamp >= vision_data.header.stamp )
                    {
                        break;
                    }
                    oit++;
                }
                throw;
            }
            oit->pose.pose.position.x = vision_data.pose.pose.position.x;
            oit->pose.pose.position.y = vision_data.pose.pose.position.y;
            break;    
        }
        cit++;
        oit++;
    }
    while( cit != vec_model_data.cend() )
    {
        // before calculate must ensure cit->stamp == oit->header.stamp
        if( cit->stamp != oit->header.stamp )
        {
            oit++;
            if( oit == vec_observer_data.end() )
            {
                std::cout   << zeabus::escape_code::bold_red << "FATAL " 
                            << zeabus::escape_code::normal_white 
                            << "REUPDATED OBSERVER : Don't have time to match observer<->model\n";
                break;
            }
            else
            {
                continue;
            }
        }
        auto mat_sum_force = cit->mat_force_gravity + 
                cit->mat_force_buoncy +
                cit->mat_force_constant +
                cit->mat_force_thruster +
                mat_force_observer;
        zeabus::math::rotation( cit->quaternion , velocity ); // rotation vector to robot frame
        // function viscosity this function reponsible by observer_parameter.cpp
        acceleration[ 0 ] = ( boost::qvm::A00( mat_sum_force ) + 
                viscosity( arr_viscosity_k[ 0 ] , arr_viscosity_c[ 0 ] , velocity[ 0 ] ) ) *
                boost::qvm::A00( zeabus::robot::mat_inertia_inverse );
        acceleration[ 1 ] = ( boost::qvm::A10( mat_sum_force ) +
                viscosity( arr_viscosity_k[ 1 ] , arr_viscosity_c[ 1 ] , velocity[ 1 ] ) ) *
                boost::qvm::A11( zeabus::robot::mat_inertia_inverse );
        acceleration[ 2 ] = ( boost::qvm::A20( mat_sum_force ) +
                viscosity( arr_viscosity_k[ 2 ] , arr_viscosity_c[ 2 ] , velocity[ 2 ] ) ) *
                boost::qvm::A22( zeabus::robot::mat_inertia_inverse );
        // now we have acceleration in base link frame
        auto diff_time = ( cit->stamp - stamp ).toSec();
        velocity[ 0 ] += acceleration[ 0 ] * diff_time;
        velocity[ 1 ] += acceleration[ 1 ] * diff_time;
        velocity[ 2 ] += acceleration[ 2 ] * diff_time;
        velocity[ 2 ] = 0;
        // convert data from base line to odom
        zeabus::math::inv_rotation( cit->quaternion , velocity );
        zeabus::math::inv_rotation( cit->quaternion , acceleration );
        position[ 0 ] += velocity[ 0 ] * diff_time - 
                acceleration[ 0 ] * pow( diff_time , 2 ) / 2.0;
        position[ 1 ] += velocity[ 1 ] * diff_time - 
                acceleration[ 1 ] * pow( diff_time , 2 ) / 2.0;
        oit->pose.pose.position.x = position[0];
        oit->pose.pose.position.y = position[1];
        stamp = cit->stamp;
        cit++;
        oit++; 
    }

    // copy velocity in x y odom frame
    std::memcpy( (void*) arr_odom_linear_velocity.c_array() ,
            (void*) velocity , sizeof( double ) * 2 );
    // Main observer will rotate data to base link before calculate model in file observer.cpp

end_function_reupdate_velocity:
    return have_reupdate;
} // function reupdate_calculate

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
