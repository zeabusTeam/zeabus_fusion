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
#define _TUNE_VISION_OBSERVER_EQUATION_
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

#ifndef _TUNE_VISION_OBSERVER_EQUATION_
const double learning_rate_observer = 0.1;
#else
const double learning_rate_observer = 0.05;
#endif
const double learning_rate_k = 0.001;
const double learning_rate_c = 0.001;
const double minimum_cost_value = 0.05;
const double maximum_cost_value = 500;

const std::string package_name = "zeabus_localize";
const std::string sub_directory = "parameter/observer";
const std::string file_name = "default.csv";
const std::string file_name_dump = "end_02.csv";
const std::string file_name_load = "start_02.csv";

void load_parameter()
{
    zeabus::FileCSV fh;
    fh.open( zeabus_ros::get_full_path( package_name , sub_directory , file_name_load ) );
    zeabus::extract_csv_type_3( &fh , &mat_force_observer.a[0][0] , 
            arr_viscosity_k.c_array(),
            arr_viscosity_c.c_array() );
    fh.close();
}

void dump_parameter()
{
    std::string temp = zeabus_ros::get_full_path( package_name , sub_directory , file_name_dump ); 
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
        double temp;
        auto sum_x = boost::qvm::A00( temp_sum ) + 
                viscosity( arr_viscosity_k[ 0 ] , arr_viscosity_c[ 0 ] , vec_vision_velocity_1.x() );
        auto sum_y = boost::qvm::A10( temp_sum ) +
                viscosity( arr_viscosity_k[ 1 ] , arr_viscosity_c[ 1 ] , vec_vision_velocity_1.y() );
        auto sum_z = boost::qvm::A20( temp_sum ) +
                viscosity( arr_viscosity_k[ 2 ] , arr_viscosity_c[ 2 ] , vec_vision_velocity_1.z() );

        auto accel_x = sum_x / boost::qvm::A00( zeabus::robot::mat_inertia );
        auto accel_y = sum_y / boost::qvm::A11( zeabus::robot::mat_inertia );
        auto accel_z = sum_z / boost::qvm::A22( zeabus::robot::mat_inertia );

        // Temp variable use to collect data when velocity is position
        // viscosity when velocity is position return negative 
#ifdef _USE_VISCOSITY_EXPONENTIAL_ 
        temp = -1 + exp( -1.0 * arr_viscosity_c[ 0 ] * fabs( vec_vision_velocity_1.x() ) ); // vel +
        if( vec_vision_velocity_1.x() < 0 ) temp *= -1.0; 
        auto new_k_x = arr_viscosity_k[ 0 ] - learning_rate_k * 
                ( vec_vision_acceleration.x() - accel_x ) /
                ( boost::qvm::A00( zeabus::robot::mat_inertia) ) * temp;

        temp = -1 + exp( -1.0 * arr_viscosity_c[ 1 ] * fabs( vec_vision_velocity_1.y() ) ); // vel +
        if( vec_vision_velocity_1.y() < 0 ) temp *= -1.0; 
        auto new_k_y = arr_viscosity_k[ 1 ] - learning_rate_k * 
                ( vec_vision_acceleration.y() - accel_y ) /
                ( boost::qvm::A11( zeabus::robot::mat_inertia) ) * temp;

        temp = -1 + exp( -1.0 * arr_viscosity_c[ 2 ] * fabs( vec_vision_velocity_1.z() ) ); // vel +
        if( vec_vision_velocity_1.z() < 0 ) temp *= -1.0; 
        auto new_k_z = arr_viscosity_k[ 2 ] - learning_rate_k * 
                ( vec_vision_acceleration.z() - accel_z ) /
                ( boost::qvm::A22( zeabus::robot::mat_inertia) ) * temp;

        temp = -1.0 * arr_viscosity_k[ 0 ] * fabs( vec_vision_velocity_1.x() ) * 
                exp( -1.0 * arr_viscosity_c[ 0 ] * fabs( vec_vision_velocity_1.x() ) ); 
        if( vec_vision_velocity_1.x() < 0 ) temp *= -1;
        auto new_c_x = arr_viscosity_c[ 0 ] - learning_rate_c *
                ( vec_vision_acceleration.x() - accel_x ) /
                boost::qvm::A00( zeabus::robot::mat_inertia ) * temp;

        temp = -1.0 * arr_viscosity_k[ 1 ] * fabs( vec_vision_velocity_1.y() ) * 
                exp( -1.0 * arr_viscosity_c[ 1 ] * fabs( vec_vision_velocity_1.y() ) ); 
        if( vec_vision_velocity_1.y() < 0 ) temp *= -1;
        auto new_c_y = arr_viscosity_c[ 2 ] - learning_rate_c *
                ( vec_vision_acceleration.y() - accel_y ) /
                boost::qvm::A11( zeabus::robot::mat_inertia ) * temp;

        temp = -1.0 * arr_viscosity_k[ 2 ] * fabs( vec_vision_velocity_1.z() ) * 
                exp( -1.0 * arr_viscosity_c[ 2 ] * fabs( vec_vision_velocity_1.z() ) ); 
        if( vec_vision_velocity_1.z() < 0 ) temp *= -1;
        auto new_c_z = arr_viscosity_c[ 2 ] - learning_rate_c *
                ( vec_vision_acceleration.z() - accel_z ) /
                boost::qvm::A22( zeabus::robot::mat_inertia ) * temp;
#else // condition _USE_VISCOSITY_LINEAR
        auto new_k_x = arr_viscosity_k[ 0 ] - learning_rate_k * 
                ( vec_vision_acceleration.x() - accel_x ) /
                boost::qvm::A00( zeabus::robot::mat_inertia ) *
                ( -1.0 * vec_vision_velocity_1.x() );
        auto new_k_y = arr_viscosity_k[ 1 ] - learning_rate_k * 
                ( vec_vision_acceleration.y() - accel_y ) /
                boost::qvm::A11( zeabus::robot::mat_inertia ) *
                ( -1.0 * vec_vision_velocity_1.y() );
        auto new_k_z = arr_viscosity_k[ 2 ] - learning_rate_k * 
                ( vec_vision_acceleration.z() - accel_z ) /
                boost::qvm::A22( zeabus::robot::mat_inertia ) *
                ( -1.0 * vec_vision_velocity_1.y() );
        auto new_c_x = arr_viscosity_c[ 0 ];
        auto new_c_y = arr_viscosity_c[ 1 ];
        auto new_c_z = arr_viscosity_c[ 2 ];
#endif // end confition calculate

#ifndef _TUNE_VISION_OBSERVER_EQUATION_
        auto new_observer_force_x = boost::qvm::A00( mat_force_observer ) -learning_rate_observer *
                ( vec_vision_acceleration.x() - accel_x ) / 
                boost::qvm::A00( zeabus::robot::mat_inertia );
        auto new_observer_force_y = boost::qvm::A10( mat_force_observer ) -learning_rate_observer *
                ( vec_vision_acceleration.y() - accel_y ) / 
                boost::qvm::A11( zeabus::robot::mat_inertia );
        auto new_observer_force_z = boost::qvm::A20( mat_force_observer ) -learning_rate_observer *
                ( vec_vision_acceleration.z() - accel_z ) / 
                boost::qvm::A22( zeabus::robot::mat_inertia );
#else
        auto want_observer_force_x = 
                vec_vision_acceleration.x() * boost::qvm::A00( zeabus::robot::mat_inertia ) - 
                sum_x + boost::qvm::A00( mat_force_observer );
        auto want_observer_force_y = 
                vec_vision_acceleration.y() * boost::qvm::A11( zeabus::robot::mat_inertia ) - 
                sum_y + boost::qvm::A10( mat_force_observer );
        auto want_observer_force_z = 
                vec_vision_acceleration.z() * boost::qvm::A22( zeabus::robot::mat_inertia ) -
                sum_z + boost::qvm::A20( mat_force_observer );
        // new  = origin + learning * ( desire - origin ) 
        //      = ( origin - 1 )*learning + learning * desire
        //      = learning * ( origin - 1 + desire );
        auto new_observer_force_x = learning_rate_observer * (
                boost::qvm::A00( mat_force_observer ) - 1 + want_observer_force_x );
        auto new_observer_force_y = learning_rate_observer * (
                boost::qvm::A10( mat_force_observer ) - 1 + want_observer_force_y );
        auto new_observer_force_z = learning_rate_observer * (
                boost::qvm::A20( mat_force_observer ) - 1 + want_observer_force_z );
#endif

        double cost_x = pow( vec_vision_acceleration.x() - accel_x , 2 );
        double cost_y = pow( vec_vision_acceleration.y() - accel_y , 2 );
        double cost_z = pow( vec_vision_acceleration.z() - accel_z , 2 );
#ifdef _PRINT_TUNE_VISION_
        printf( "------------- TUNE OBSERVER PART VISION --------------------------\n" );
        printf( "Cost Value :%10.3f%10.3f%10.3f\n" , cost_x , cost_y , cost_z ); 
        printf( "k_constant :%10.2f%10.2f%10.2f   to%10.2f%10.2f%10.2f\n" , 
                arr_viscosity_k[ 0 ], arr_viscosity_k[ 1 ], arr_viscosity_k[ 2 ],
                new_k_x , new_k_y , new_k_z );
#ifdef _USE_VISCOSITY_EXPONENTIAL_
        printf( "c_constant :%10.2f%10.2f%10.2f   to%10.2f%10.2f%10.2f\n" , 
                arr_viscosity_c[ 0 ], arr_viscosity_c[ 1 ], arr_viscosity_c[ 2 ],
                new_c_x , new_c_y , new_c_z );
#endif // _USE_VISCOSITY_EXPONENTIAL_
#ifndef _TUNE_VISION_OBSERVER_EQUATION_
        printf( "f_constant :%10.2f%10.2f%10.2f   to%10.2f%10.2f%10.2f\n\n" ,
                boost::qvm::A00( mat_force_observer ) , 
                boost::qvm::A10( mat_force_observer ) ,
                boost::qvm::A20( mat_force_observer ) ,
                new_observer_force_x , new_observer_force_y , new_observer_force_z );
#else
        printf( "f_constant :%8.2f%8.2f%8.2f want %8.2f%8.2f%8.2f get %8.2f%8.2f%8.f\n" ,
                boost::qvm::A00( mat_force_observer ) , 
                boost::qvm::A10( mat_force_observer ) ,
                boost::qvm::A20( mat_force_observer ) ,
                want_observer_force_x , want_observer_force_y , want_observer_force_z, 
                new_observer_force_x , new_observer_force_y , new_observer_force_z );
#endif // _TUNE_VISION_OBSERVER_EQUATION_
#endif // _PRINT_TUNE_VISION_
        if( std::isnan( cost_x ) || std::isnan( cost_y ) || std::isnan( cost_z ) )
        {
            std::cout   << zeabus::escape_code::bold_yellow << "WARNING "
                        << zeabus::escape_code::normal_white 
                        << "PARAMETER OBSERVER : Can't edit parameter because NaN(Not a Number)\n";
        }
        if( cost_x < minimum_cost_value || cost_x > maximum_cost_value )
        {
            ;
        }
        else
        {
            arr_viscosity_c[ 0 ] = new_c_x;
            arr_viscosity_k[ 0 ] = new_k_x;
            boost::qvm::A00( mat_force_observer ) = new_observer_force_x;
        }

        if( cost_y < minimum_cost_value || cost_y > maximum_cost_value )
        {
            ;
        }
        else
        {
            arr_viscosity_c[ 1 ] = new_c_y;
            arr_viscosity_k[ 1 ] = new_k_y;
            boost::qvm::A10( mat_force_observer ) = new_observer_force_y;
        }

        if( cost_z < minimum_cost_value || cost_z > maximum_cost_value )
        {
            ;
        }
        else
        {
            arr_viscosity_c[ 2 ] = new_c_z;
            arr_viscosity_k[ 2 ] = new_k_z;
            boost::qvm::A20( mat_force_observer ) = new_observer_force_z;
        }
        // not use 1 because data know acceleration and data vision data have velocity in unit 2
        if( ! reupdate_calculate( vision_data , vec_vision_velocity_2 ) )
        {
            std::cout   << "CONFIG INREGRAL ON VISION =======================================\n";
            b_config_integral_vision = true;
        }
        else
        {
/*            printf( "ORIGINAL %8.3f,%8.3f\n" , observer_data.pose.pose.position.x,
                    observer_data.pose.pose.position.y );
            printf( "NEW      %8.3f,%8.3f\n" , (vec_observer_data.cend()-1)->pose.pose.position.x,
                    (vec_observer_data.cend()-1)->pose.pose.position.y );*/
            observer_data.pose.pose.position.x = (vec_observer_data.cend()-1)->pose.pose.position.x;
            observer_data.pose.pose.position.y = (vec_observer_data.cend()-1)->pose.pose.position.y;
        }
    } // condition hit data in buffer
    else
    {
        std::cout   << "TUNE OBSERVER : Don't have data suitable in buffer\n";
    }
exit_config_parameter_on_vision:
    return;    
} // config_parameter_on_vision

void config_parameter_on_localize_angular()
{
    ;
} // config_parameter_on_localize_angular

void active_parameter()
{
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
#ifdef _TUNE_VISION_
    if( b_config_model_vision )
    {
        config_parameter_on_vision(); 
        b_config_model_vision = false;
    }
#else
    if( ! reupdate_calculate( vision_data , vec_vision_velocity_2 ) )
    {
        std::cout   << "CONFIG INREGRAL ON VISION =======================================\n";
        b_config_integral_vision = true;
    }
    else
    {
        observer_data.pose.pose.position.x = vec_observer_data.crend()->pose.pose.position.x;
        observer_data.pose.pose.position.y = vec_observer_data.crend()->pose.pose.position.y;
    }
#endif 
    return;
} // active_parameter

inline double viscosity( const double k , const double c , const double velocity )
{
#ifdef _USE_VISCOSITY_EXPONENTIAL_
    double answer = k * ( 1 - exp( -1.0 * c * fabs( velocity ) ) );
    if( velocity > 0 ) answer *= -1;
    return answer;
#else
    return -1.0 * k * velocity;
#endif
} // viscosity
