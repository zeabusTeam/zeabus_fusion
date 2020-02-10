// FILE			: mapper_reconfigure.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 10 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <mapper_reconfigure.hpp>

double mass = 0 ;         
double moment_inertia = 0 ;
double volumn = 0 ;          
double viscosty[ 6 ];
bool avaliable_new_parameter = false;
std::mutex lock_dynamic_reconfigure;

void dynamic_reconfigure_callback( zeabus_localize::VisualConfig &config , uint32_t level )
{
    std::cout   << "Dynamic reconfigure callback " << level << "\n";

    lock_dynamic_reconfigure.lock();
    mass = config.mass;
    moment_inertia = config.moment_inertia;
    volumn = config.volumn;
    viscosty[ 0 ] = config.viscosty_x;
    viscosty[ 1 ] = config.viscosty_y;
    viscosty[ 2 ] = config.viscosty_z;
    viscosty[ 3 ] = config.viscosty_roll;
    viscosty[ 4 ] = config.viscosty_pitch;
    viscosty[ 5 ] = config.viscosty_yaw;
    avaliable_new_parameter = true;
    lock_dynamic_reconfigure.unlock();
}
