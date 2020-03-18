// FILE			: observer_localize.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 09 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

extern bool b_config_model_z_linear;
extern bool b_config_model_rotation;
extern bool b_config_integral_rotation;
extern bool b_config_integral_z_linear;
extern bool b_start_active_z;
extern bool b_start_active_rotation;
extern double mean_z;
extern double max_z;
extern double min_z;

#ifndef _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_LOCALIZE_HPP__
#define _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_LOCALIZE_HPP__
void reset_localize();
void active_localize();
void active_rotation();
void reset_rotation();
void active_z();
void reset_z();
#endif // _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_LOCALIZE_HPP__
