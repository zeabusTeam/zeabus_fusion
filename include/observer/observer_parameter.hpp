// FILE			: observer_parameter.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 09 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

extern boost::qvm::mat< double , 6 , 1 > mat_force_observer;
extern boost::array< double , 6 > arr_viscosity_k;
extern boost::array< double , 6 > arr_viscosity_c;

#ifndef _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_PARAMETER_HPP__
#define _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_PARAMETER_HPP__
void load_parameter();
void dump_parameter();
void active_parameter();
void active_vision();
void active_localize();
#endif // _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_PARAMETER_HPP__ 
