// FILE			: observer_parameter.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
//#define _TUNE_ALL_
#define _TUNE_VISION_
//#define _TUNE_LOCALIZE_

// MACRO CONDITION
#ifdef _TUNE_ALL_
    #define _TUNE_VISION_
    #define _TUNE_LOCALIZE_
#endif

#include    <observer/observer.hpp>

boost::array< double , 6 > arr_viscosity_k = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::array< double , 6 > arr_viscosity_c = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 6 , 1 > mat_force_observer = { 0 , 0 , 0 , 0 , 0 , 0 };

const double learning_rate = 0.1;

const std::string package_name = "zeabus_localize";
const std::string sub_directory = "parameter/observer";
const std::string file_name = "default.csv";
const std::string file_name_dump = file_name;
const std::string file_name_save = file_name;

void load_parameter()
{
    zeabus::FileCSV fh;
    fh.open( zeabus_ros::get_full_path( package_name , sub_directory , file_name ) );
    zeabus::extract_csv_type_3( &fh , &mat_force_observer.a[0][0] , 
            arr_viscosity_k.c_array(),
            arr_viscosity_c.c_array() );
    fh.close();
}

void dump_parameter()
{
    std::string temp = zeabus_ros::get_full_path( package_name , sub_directory , file_name ); 
    zeabus::dump_csv_type_3( temp, 
            &mat_force_observer.a[0][0],
            arr_viscosity_k.c_array(),
            arr_viscosity_c.c_array(),
            6
    );
}

void config_parameter_on_vision()
{
    DataObserverModel temp_data;
    if( search_buffer_model( vision_data.header.stamp , &temp_data ) )
    {
        auto temp_sum = temp_data.mat_force_gravity + 
                temp_data.mat_force_buoncy +
                temp_data.mat_force_constant +
                temp_data.mat_force_thruster +
                mat_force_observer;
    }
    else
    {
        std::cout   << "TUNE OBSERVER : Don't have data suitable in buffer\n";
    }
    
} // config_parameter_on_vision

void config_parameter_on_localize_angular()
{

} // config_parameter_on_localize_angular

void active_parameter()
{
#ifdef _TUNE_VISION_
    if( b_config_model_vision )
    {
       config_parameter_on_localize_angular(); 
    }
#endif    
#ifdef _TUNE_LOCALIZE_
    // We accept model parameter from camera more than pressure sensor differential
    if( b_config_model_z_linear && !( b_config_model_vision ) )
    {
        ;
    }
    if( b_config_model_rotation )
    {
        config_parameter_on_localize_angular();
    }
#endif
#ifndef _TUNE_VISION_
#ifndef _TUNE_LOCALIZE_
    std::cout   << "Warning you didn't allow to config mode observer\n";
#endif // _TUNE_LOCALIZE_
#endif // _TUNE_VISION_
    return;
} // active_parameter

inline double viscosity( const double k , const double c , const double velocity )
{
    return -1.0 * k * ( 1 - exp( -1.0 * c * velocity ) );
} // viscosity
