// FILE			: observer_bound_limit.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 09 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

extern const double global_max_z;
extern const double global_min_z;
extern const double global_diff_z;
extern const double global_diff_z_half;
extern const double global_active_observer;
extern bool observer_status;

#ifndef _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_BOUND_LIMIT_HPP__
#define _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_BOUND_LIMIT_HPP__
void report_bound_limit();
void active_bound_limit();
void active_bound_limit_depth();
#endif // _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_BOUND_LIMIT_HPP__
