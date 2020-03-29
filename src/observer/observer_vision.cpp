// FILE			: observer_vision.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 09 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <observer/observer.hpp>

bool b_config_integral_vision = false;
bool b_config_model_vision = false;
const double limit_time_vision = 1.0;

tf::Vector3 vec_vision_velocity_1;
tf::Vector3 vec_vision_velocity_2;
tf::Vector3 vec_vision_acceleration;

std::vector< DataRosVision > vec_vision_data;

void active_vision()
{
    
    double temp;
    vec_vision_data.push_back( DataRosVision( &vision_data ) );

    switch( vec_vision_data.size() )
    {
    case 3 : // can find acceleration
        b_config_model_vision = true;
        temp = ( vec_vision_data[2].stamp - vec_vision_data[1].stamp ).toSec();
        vec_vision_velocity_2 = ( vec_vision_data[ 2 ].pose - vec_vision_data[ 1 ].pose ) / temp; 
        vec_vision_acceleration = ( vec_vision_velocity_2 - vec_vision_velocity_1 ) / temp;

    case 2 : // can find velocity in odom frame
        vec_vision_velocity_1 = ( vec_vision_data[ 1 ].pose - vec_vision_data[ 0 ].pose ) / 
                ( vec_vision_data[1].stamp - vec_vision_data[0].stamp ).toSec();

    case 1 : // can update position in integral vision part
        b_config_model_vision = true; // because you will cal when have new datail
        break;

    case 0 :
        std::cout   << "VISION OBSERVER : Don't have data in buffer!!!\n";
        break;

    default:
        std::cout   << "VISION OBSERVER : Vision Have data overload\n";
    }
    
}

void reset_vision()
{
    b_config_model_vision = false;
    b_config_integral_vision = false;
    vec_vision_velocity_1.setValue( 0 , 0 , 0 );
    vec_vision_velocity_2.setValue( 0 , 0 , 0 );
    vec_vision_acceleration.setValue( 0 , 0 , 0 );
    vec_vision_data.clear();
}

void check_buffer_vision()
{
    while( vec_vision_data.size() > 3 )
    {
        vec_vision_data.erase( vec_vision_data.begin() );
    }

    while( vec_vision_data.size() > 0 )
    {
        if( (vision_data.header.stamp - vec_vision_data.at(0).stamp ).toSec() > limit_time_vision )
        {
            vec_vision_data.erase( vec_vision_data.begin() );
        }
        else
        {
            break;
        }
    }
}
