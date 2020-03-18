// FILE			: observer_model.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 10 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

extern boost::qvm::mat< double , 6 , 1 > mat_acceleration;
extern boost::qvm::mat< double , 6 , 1 > mat_force_gravity;
extern boost::qvm::mat< double , 6 , 1 > mat_force_buoncy;
extern boost::qvm::mat< double , 6 , 1 > mat_force_constant;
extern boost::qvm::mat< double , 6 , 1 > mat_force_viscosity;
extern double roll;
extern double pitch;
extern double yaw;

#ifndef _CPE_PROJECT_ZEABUS_LOCALIZE_OBSERVER_OBSERVER_MODEL_HPP__
#define _CPE_PROJECT_ZEABUS_LOCALIZE_OBSERVER_OBSERVER_MODEL_HPP__
void report_model();
void active_model();
void calculate_viscosity();
double viscosity( unsigned int index );
#endif // _CPE_PROJECT_ZEABUS_LOCALIZE_OBSERVER_OBSERVER_MODEL_HPP__
