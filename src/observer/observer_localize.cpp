// FILE			: observer_localize.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 09 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <observer/observer.hpp>

bool b_config_model_z_linear = false;
bool b_config_model_rotation = false;
bool b_config_integral_rotation = false;
bool b_config_integral_z_linear = false;
bool b_start_active_z = false;
bool b_start_active_rotation = false;
double mean_z = 0;
double max_z = 0;
double min_z = 0;

void active_localize()
{
    active_rotation();
    active_z();
} 

void active_rotation()
{
    ;     
} 

void reset_rotation()
{
    ;
}

void active_z( )
{
    if( ! b_start_active_z )
    {
        min_z = localize_data.pose.pose.position.z - global_diff_z_half;
        max_z = localize_data.pose.pose.position.z + global_diff_z_half;
        mean_z = localize_data.pose.pose.position.z;
        observer_data.pose.pose.position.z = localize_data.pose.pose.position.z;
        b_start_active_z = true;
        goto exit_active_z;
    }
    
    if( fabs( localize_data.pose.pose.position.z - mean_z ) > global_diff_z * 2 )
    { // filter this value because estimate this is noise
        goto exit_active_z;
    }
    else
    {
        b_config_model_z_linear = true;
        mean_z = localize_data.pose.pose.position.z;
        max_z = mean_z + global_diff_z_half;
        min_z = mean_z - global_diff_z_half;
    }
exit_active_z:
    return;
}

void reset_localize()
{
    reset_z();
    reset_rotation();
}

void reset_z()
{
    b_config_integral_z_linear = false;
    b_config_model_z_linear = false;
    b_start_active_z = false;
    mean_z = 0;
    max_z = 0;
    min_z = 0;
}
