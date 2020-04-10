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
    static tf::Transformer tf_transform; 
    double temp;
    vec_vision_data.push_back( DataRosVision( &vision_data ) );
    check_buffer_vision();
    switch( vec_vision_data.size() )
    {
    case 3 : // can find acceleration
        b_config_model_vision = true;
        temp = ( vec_vision_data[2].stamp - vec_vision_data[1].stamp ).toSec();
        vec_vision_velocity_2 = ( vec_vision_data[ 2 ].pose - vec_vision_data[ 1 ].pose ) / temp; 
        vec_vision_velocity_1 = ( vec_vision_data[ 1 ].pose - vec_vision_data[ 0 ].pose ) / 
                ( vec_vision_data[1].stamp - vec_vision_data[0].stamp ).toSec();
        vec_vision_acceleration = ( vec_vision_velocity_2 - vec_vision_velocity_1 ) / temp;
        break;
    case 2 : // can find velocity in odom frame
        vec_vision_velocity_1 = ( vec_vision_data[ 1 ].pose - vec_vision_data[ 0 ].pose ) / 
                ( vec_vision_data[1].stamp - vec_vision_data[0].stamp ).toSec();
        if( ! reupdate_calculate( vision_data , vec_vision_velocity_1 ) )
        {
            std::cout   << "CONFIG INREGRAL ON VISION =======================================\n";
            b_config_integral_vision = true;
        }
        break;
    case 1 : // can update position in integral vision part
        if( ! reupdate_position( vision_data ) )
        {
            std::cout   << "CONFIG INREGRAL ON VISION =======================================\n";
            b_config_integral_vision = true; // because you will cal when have new data
        }
        else
        {
            ; // you update position in middle data, not last data!
        }
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
