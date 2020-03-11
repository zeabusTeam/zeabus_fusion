// FILE			: observer_model.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 09 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <observer/observer.hpp>

boost::qvm::mat< double , 6 , 1 > mat_acceleration = {  0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 6 , 1 > mat_force_gravity = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 6 , 1 > mat_force_buoncy = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 6 , 1 > mat_force_constant = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 6 , 1 > mat_force_viscosity = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< >
double roll = 0;
double pitch = 0;
double yaw = 0;

void active_model()
{
    boost::qvm::mat< double , 3 , 3 > mat_rotation_force_z = {
        0,  0,  -1.0*sin( roll ),
        0,  0,  cos( pitch ) * sin( roll ),
        0,  0,  cos( pitch ) * cos( roll )
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
                    zeabus::robot::mat_center_buoncy );

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
}

inline void calculate_viscosity()
{
    boost::qvm::A00( mat_force_viscosity ) = viscosity( 0 );
    boost::qvm::A10( mat_force_viscosity ) = viscosity( 1 );
    boost::qvm::A20( mat_force_viscosity ) = viscosity( 2 );
    boost::qvm::A30( mat_force_viscosity ) = viscosity( 3 );
    boost::qvm::A40( mat_force_viscosity ) = viscosity( 4 );
    boost::qvm::A50( mat_force_viscosity ) = viscosity( 5 );
    boost::qvm::A60( mat_force_viscosity ) = viscosity( 6 );
    boost::qvm::A70( mat_force_viscosity ) = viscosity( 7 );
}

inline double viscosity( unsigned int index )
{
    return -1.0 * arr_viscosity_k[ index ] * 
            ( 1 - exp( -1.0 * arr_viscosity_c[index] * arr_robot_velocity[ index ] ) );
}

void report_model()
{
    printf( "Gravity force   :" ); zeabus_boost::printT( mat_force_gravity ); 
    printf( "Buoyancy force  :" ); zeabus_boost::printT( mat_force_buoncy );
    printf( "Constant force  :" ); zeabus_boost::printT( mat_force_constant ); 
    printf( "Viscosity force :" ); zeabus_boost::printT( mat_force_viscosity );
    printf( "Thruster force  :" ); zeabus_boost::printT( mat_force_thruster );
    printf( "Observer force  :" ); zeabus_boost::printT( mat_force_observer );
}
