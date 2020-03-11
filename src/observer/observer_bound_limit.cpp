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

void bound_limit_parameter()
{
    printf( "BOUND_LIMIT : Global min & max :%8.3f & %8.3f\n" , global_min_z , global_max_z );
    printf( "BOUND_LIMIT : Global diff and activate :%8.3f & %8.3f\n" , 
            global_diff_z , global_active_observer );
}

void bound_limit_module()
{
    ;
}

void bound_limit_depth()
{
    ;
}
