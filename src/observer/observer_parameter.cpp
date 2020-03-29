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

const std::string package_name = "zeabus_localize";
const std::string sub_directory = "parameter/observer";
const std::string file_name = "default.csv";

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

void active_parameter()
{
#ifdef _TUNE_VISION_
    if( b_config_model_vision )
    {
       ; 
    }
#endif    
#ifdef _TUNE_LOCALIZE_
    if( b_config_model_z_linear )
    {
        ;
    }
    if( b_config_model_rotation )
    {
        ;
    }
#endif
}
