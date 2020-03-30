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
#define _PRINT_TUNE_VISION_

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
        // Data before use to calculate have to base link frame
        zeabus::math::rotation( temp_data.quaternion , &vec_vision_velocity_1 );
        zeabus::math::rotation( temp_data.quaternion , &vec_vision_acceleration );
        // Don't worry to send ptr data because in each loop will calculate again every loop
        double sum_x = boost::qvm::A00( temp_sum ) + 
                viscosity( arr_viscosity_k[ 0 ] , arr_viscosity_c[ 0 ] , vec_vision_velocity_1.x() );
        double sum_y = boost::qvm::A10( temp_sum ) +
                viscosity( arr_viscosity_k[ 1 ] , arr_viscosity_c[ 1 ] , vec_vision_velocity_1.y() );
        double sum_z = boost::qvm::A20( temp_sum ) +
                viscosity( arr_viscosity_k[ 2 ] , arr_viscosity_c[ 2 ] , vec_vision_velocity_1.z() );

        double accel_x = sum_x / boost::qvm::A00( zeabus::robot::mat_inertia );
        double accel_y = sum_y / boost::qvm::A11( zeabus::robot::mat_inertia );
        double accel_z = sum_z / boost::qvm::A22( zeabus::robot::mat_inertia );
    
        double new_k_x = arr_viscosity_k[ 0 ] - learning_rate * 
                ( vec_vision_acceleration.x() - accel_x ) /
                ( boost::qvm::A00( zeabus::robot::mat_inertia) ) *
                ( 1 - exp( -1.0 * arr_viscosity_c[ 0 ] * vec_vision_velocity_1.x() ) );
        double new_k_y = arr_viscosity_k[ 1 ] - learning_rate * 
                ( vec_vision_acceleration.y() - accel_y ) /
                ( boost::qvm::A11( zeabus::robot::mat_inertia) ) *
                ( 1 - exp( -1.0 * arr_viscosity_c[ 1 ] * vec_vision_velocity_1.y() ) );
        double new_k_z = arr_viscosity_k[ 2 ] - learning_rate * 
                ( vec_vision_acceleration.z() - accel_z ) /
                ( boost::qvm::A22( zeabus::robot::mat_inertia) ) *
                ( 1 - exp( -1.0 * arr_viscosity_c[ 2 ] * vec_vision_velocity_1.z() ) );

        double new_c_x = arr_viscosity_c[ 0 ] - learning_rate *
                ( vec_vision_acceleration.x() - accel_x ) /
                boost::qvm::A00( zeabus::robot::mat_inertia ) *
                arr_viscosity_k[ 0 ] * vec_vision_velocity_1.x() * 
                exp( -1.0 * arr_viscosity_c[ 0 ] * vec_vision_velocity_1.x() );
        double new_c_y = arr_viscosity_c[ 2 ] - learning_rate *
                ( vec_vision_acceleration.y() - accel_y ) /
                boost::qvm::A11( zeabus::robot::mat_inertia ) *
                arr_viscosity_k[ 1 ] * vec_vision_velocity_1.y() * 
                exp( -1.0 * arr_viscosity_c[ 1 ] * vec_vision_velocity_1.y() );
        double new_c_z = arr_viscosity_c[ 2 ] - learning_rate *
                ( vec_vision_acceleration.z() - accel_z ) /
                boost::qvm::A22( zeabus::robot::mat_inertia ) *
                arr_viscosity_k[ 2 ] * vec_vision_velocity_1.z() * 
                exp( -1.0 * arr_viscosity_c[ 2 ] * vec_vision_velocity_1.z() );

        double new_observer_force_x = boost::qvm::A00( mat_force_observer ) - learning_rate *
                ( vec_vision_acceleration.x() - accel_x ) / 
                boost::qvm::A00( zeabus::robot::mat_inertia );
        double new_observer_force_y = boost::qvm::A10( mat_force_observer ) - learning_rate *
                ( vec_vision_acceleration.y() - accel_y ) / 
                boost::qvm::A11( zeabus::robot::mat_inertia );
        double new_observer_force_z = boost::qvm::A20( mat_force_observer ) - learning_rate *
                ( vec_vision_acceleration.z() - accel_z ) / 
                boost::qvm::A22( zeabus::robot::mat_inertia );

#ifdef _PRINT_TUNE_VISION_
        printf( "------------- TUNE OBSERVER PART VISION --------------------------\n" );
        printf( "k_constant :%7.2f%7.2f%7.2f to%7.2f%7.2f%7.2f\n" , 
                arr_viscosity_k[ 0 ], arr_viscosity_k[ 1 ], arr_viscosity_k[ 2 ],
                new_k_x , new_k_y , new_k_z );
        printf( "c_constant :%7.2f%7.2f%7.2f to%7.2f%7.2f%7.2f\n" , 
                arr_viscosity_c[ 0 ], arr_viscosity_c[ 1 ], arr_viscosity_c[ 2 ],
                new_c_x , new_c_y , new_c_z );
        printf( "f_constant :%10.2f%10.2f%10.2f to%10.2f%10.2f%10.2f\n\n" ,
                boost::qvm::A00( mat_force_observer ) , 
                boost::qvm::A10( mat_force_observer ) ,
                boost::qvm::A20( mat_force_observer ) ,
                new_observer_force_x , new_observer_force_y , new_observer_force_z );
#endif // _PRINT_TUNE_VISION_
        arr_viscosity_c[ 0 ] = new_c_x;
        arr_viscosity_c[ 1 ] = new_c_y;
        arr_viscosity_c[ 2 ] = new_c_z;
        arr_viscosity_k[ 0 ] = new_k_x;
        arr_viscosity_k[ 1 ] = new_k_y;
        arr_viscosity_k[ 2 ] = new_k_z;
        boost::qvm::A00( mat_force_observer ) = new_observer_force_x;
        boost::qvm::A10( mat_force_observer ) = new_observer_force_y;
        boost::qvm::A20( mat_force_observer ) = new_observer_force_z;
    }
    else
    {
        std::cout   << "TUNE OBSERVER : Don't have data suitable in buffer\n";
    }
    
} // config_parameter_on_vision

void config_parameter_on_localize_angular()
{
    ;
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
