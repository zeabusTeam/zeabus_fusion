// FILE			: observer_model.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 09 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
// #define _PRINT_MODEL_

// MACRO CONDITION

#include    <observer/observer.hpp>

boost::qvm::mat< double , 6 , 1 > mat_acceleration = {  0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 6 , 1 > mat_force_gravity = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 6 , 1 > mat_force_buoncy = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 6 , 1 > mat_force_constant = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 6 , 1 > mat_force_viscosity = { 0 , 0 , 0 , 0 , 0 , 0 };
double roll = 0;
double pitch = 0;
double yaw = 0;

std::vector< DataObserverModel > vec_model_data;

void active_model( const ros::Time& current_time )
{

    zeabus::math::get_euler( current_quaternion , &roll , &pitch , &yaw );
    boost::qvm::mat< double , 3 , 3 > mat_rotation_force_z = {
        0,  0,  -1.0*sin( roll ),
        0,  0,  cos( pitch ) * sin( roll ),
        0,  0,  cos( pitch ) * cos( roll )
//        0,  0,  0,
//        0,  0,  0,
//        -1.0*sin(roll),  cos( pitch) * sin( roll ),  cos( pitch ) * cos( roll )
    };

    zeabus_boost::mat_concat( &mat_force_gravity ,
            mat_rotation_force_z * zeabus::robot::mat_force_gravity,
            zeabus::robot::mat_center_gravity * 
                    mat_rotation_force_z * 
                    zeabus::robot::mat_force_gravity );

    zeabus_boost::mat_concat( &mat_force_buoncy,
            mat_rotation_force_z * zeabus::robot::mat_force_buoncy ,
            zeabus::robot::mat_center_buoncy * 
                    mat_rotation_force_z *
                    zeabus::robot::mat_force_buoncy );

    zeabus_boost::mat_concat( &mat_force_constant,
            mat_rotation_force_z * zeabus::robot::mat_force_constant,
            zeabus::robot::mat_center_constant *
                    mat_rotation_force_z *
                    zeabus::robot::mat_force_constant );

    calculate_viscosity();

    mat_acceleration = mat_force_gravity +
            mat_force_buoncy +
            mat_force_constant + 
            mat_force_viscosity +
            mat_force_thruster +
            mat_force_observer;

    mat_acceleration = zeabus::robot::mat_inertia_inverse * mat_acceleration;

#ifdef _PRINT_MODEL_
    report_model();
#endif // _PRINT_MODEL_

    vec_model_data.push_back( DataObserverModel( mat_acceleration , 
            mat_force_gravity,
            mat_force_buoncy,
            mat_force_constant,
            mat_force_thruster,
            arr_robot_velocity,
            current_quaternion,
            current_time ) );

    check_buffer_model( current_time - ros::Duration( global_time_limit_buffer ) );
}

void calculate_viscosity()
{
    boost::qvm::A00( mat_force_viscosity ) = viscosity( 0 );
    boost::qvm::A10( mat_force_viscosity ) = viscosity( 1 );
    boost::qvm::A20( mat_force_viscosity ) = viscosity( 2 );
    boost::qvm::A30( mat_force_viscosity ) = viscosity( 3 );
    boost::qvm::A40( mat_force_viscosity ) = viscosity( 4 );
    boost::qvm::A50( mat_force_viscosity ) = viscosity( 5 );
}

inline double viscosity( const unsigned int index )
{
    double answer = arr_viscosity_k[ index ] * 
            ( 1 - exp( -1.0 * arr_viscosity_c[index] * fabs( arr_robot_velocity[ index ] ) ) );
    if( arr_robot_velocity[ index ] > 0 ) answer *= -1;
    return answer;
}

void report_model()
{
    printf( "Velocity Model  :%8.2f%8.2f%8.2f\n" , arr_robot_velocity[ 0 ] ,
            arr_robot_velocity[ 1 ] , arr_robot_velocity[ 2 ] );
    printf( "Gravity force   :" ); zeabus_boost::printT( mat_force_gravity ); 
    printf( "Buoyancy force  :" ); zeabus_boost::printT( mat_force_buoncy );
    printf( "Constant force  :" ); zeabus_boost::printT( mat_force_constant ); 
    printf( "Viscosity force :" ); zeabus_boost::printT( mat_force_viscosity );
    printf( "Thruster force  :" ); zeabus_boost::printT( mat_force_thruster );
    printf( "Observer force  :" ); zeabus_boost::printT( mat_force_observer );
    printf( "Acceleration    :" ); zeabus_boost::printT( mat_acceleration );
}

void check_buffer_model( const ros::Time& minimum_time )
{
    while( vec_model_data.size() > 0 )
    {
        if( vec_model_data.begin()->stamp < minimum_time )
        {
            vec_model_data.erase( vec_model_data.begin() );
        }
        else
        {
            break;
        }
    }
}

void reset_buffer_model()
{
    vec_model_data.clear(); 
} // function reset_buffer_model

bool search_buffer_model( const ros::Time& target_time , DataObserverModel* ptr_data )
{
    bool have_data = false;
    for( auto rit = vec_model_data.crbegin() ; rit != vec_model_data.crend() ; rit++ )
    {
        if( rit->stamp < target_time )
        {
            *ptr_data = *rit;
            have_data = true;
        }
    }
    if( ! have_data )
    {
        std::cout   << "MODEL OBSERVER : Warning don't have data for " << target_time << "\n";  
    }
    return have_data;
} // function search_buffer_model
