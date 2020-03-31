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
extern const double learning_rate;

extern const std::string package_name;
extern const std::string sub_directory;
extern const std::string file_name;
extern const std::string file_name_dump;
extern const std::string file_name_load;

#ifndef _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_PARAMETER_HPP__
#define _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_PARAMETER_HPP__
void load_parameter();
void dump_parameter();
void active_parameter();
void config_parameter_on_vision();
void config_parameter_on_localize_angular();
inline double viscosity( const double k , const double c , const double velocity );
#endif // _ZEABUS_LOCALIZE_OBSERVER_OBSERVER_PARAMETER_HPP__ 
