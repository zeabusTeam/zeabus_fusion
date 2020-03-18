// FILE			: observer_bound_limit.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 09 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <observer/observer.hpp>

const double global_max_z = -0.1;
const double global_min_z = -4.7;
const double global_diff_z = 0.2;
const double global_diff_z_half = global_diff_z / 2.0;
const double global_active_observer = -0.2;
bool observer_status = false;

void report_bound_limit()
{
    printf( "BOUND_LIMIT : Global min & max :%8.3f & %8.3f\n" , global_min_z , global_max_z );
    printf( "BOUND_LIMIT : Global diff and activate :%8.3f & %8.3f\n" , 
            global_diff_z , global_active_observer );
}

void active_bound_limit()
{
    if( !observer_status )
    {
        if( mean_z < global_max_z && mean_z > global_min_z )
        {
            observer_status = true;
            publish( "BOUND LIMIT : Start Observer" );
        }
        else
        {
            ;
        }
    }
    else
    {
        if( mean_z > global_max_z || mean_z < global_min_z )
        {
            observer_status = false;
            publish( "BOUND LIMIT : Stop Observer" );
        }
        else
        {
            ;
        }
    }
}

void active_bound_limit_depth()
{
    if( observer_data.pose.pose.position.z > mean_z + global_diff_z )
    {
        observer_data.pose.pose.position.z = mean_z;
    }
    else if( observer_data.pose.pose.position.z > max_z )
    {
        observer_data.pose.pose.position.z = max_z;
    }
    else if( observer_data.pose.pose.position.z < mean_z - global_diff_z )
    {
        observer_data.pose.pose.position.z = mean_z;
    }
    else if( observer_data.pose.pose.position.z < min_z )
    {
        observer_data.pose.pose.position.z = min_z;
    }
    else
    {
        ;
    } 
}
