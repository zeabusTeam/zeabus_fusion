// FILE			: obsever_parameter.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
#define _OPEN_ALL_
//#define _OPEN_VISION_
//#define _OPEN_LOCALIZE_

// MACRO CONDITION
#ifdef _OPEN_ALL_
    #define _OPEN_VISION_
    #define _OPEN_LOCALIZE_
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
    zeabus::FileCSV_fh;
    fh.open( zeabus_ros::get_full_path( package_name , sub_directory , file_name ) );
    zeabus::extract_csv_type_3( &fh , mat_force_observer.a , 
            arr_viscosity_k.c_array(),
            arr_viscosity_c.c_array() );
    fh.close();
}

void dump_parameter()
{
    zeabus::dump_csv_type_3( zeabus_ros::get_full_path( package_name , sub_directory , file_name ) 
            mat_force_observer.a,
            arr_viscosity_k.c_array(),
            arr_viscosity_c.c_array(),
            6
    );
}

void active_parameter()
{
#ifdef _OPEN_VISION_
        active_vision();
#endif    
#ifdef _OPEN_LOCALIZE_
        active_localize();
#endif
}

void active_vision()
{
    ;
} // for part tune parameter or change parameter from vision data

void active_localize()
{
    ;
} // for part tune parameter or change parameter from localize < imu pressure >
