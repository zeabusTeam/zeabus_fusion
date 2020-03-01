// FILE			: mapper.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 07 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
#define SHOW_DATA
#define SHOW_ROBOT_FORCE

// MACRO CONDITION
#ifdef SHOW_DATA
    #define SHOW_ROBOT_FORCE
#endif // SHOW_DATA

#include    <mapper/calculate_by_force.hpp>

RobotForceHandle::RobotForceHandle()
{
    ;
} // function constructor

void RobotForceHandle::setup_constant( const double mass , 
        const double moment_inertia,
        const double volumn )
{
    this->weight = mass * zeabus::robot::gravity;
    this->buoncy = zeabus::robot::rho_water * zeabus::robot::gravity * volumn;
    this->mass = mass;
    this->moment_inertia = moment_inertia; 
} // function setup_constant

void RobotForceHandle::setup_ptr_data( tf::Quaternion* ptr_current_quaternion ,
        boost::qvm::mat< double , 1 , 8 >* ptr_mat_force_thruster,
        boost::qvm::vec< double , 6>* ptr_vec_current_velocity )
{
    this->ptr_current_quaternion = ptr_current_quaternion;
    this->ptr_mat_force_thruster = ptr_mat_force_thruster;
    this->ptr_vec_current_velocity = ptr_vec_current_velocity;
} // function setup_ptr_data

void RobotForceHandle::setup_viscosty( const boost::qvm::vec< double , 6 > vec_constant_viscosty )
{
    boost::qvm::assign( this->vec_constant_viscosty , vec_constant_viscosty );
    this->vec_constant_viscosty *= -1.0;
} // function setup_viscosty

void RobotForceHandle::calculate( const double diff_time )
{
#ifdef SHOW_DATA
    std::cout   << "CURRENT_FORCE  :\n" ;
    zeabus_boost::print( *( this->ptr_mat_force_thruster ) );
#endif // SHOW_DATA
    tf::Quaternion temp_quaternion;
    // First I will prepare data of buoncy force and weight
    temp_quaternion = tf::Quaternion( 0 , 0 , this->buoncy , 0 ); // Load buoncy force odom frame
    temp_quaternion = ptr_current_quaternion->inverse() * 
            temp_quaternion * 
            (*ptr_current_quaternion); // rotation force
    this->vec_force_buoncy.a[ 0 ] = temp_quaternion.x(); // save x axis
    this->vec_force_buoncy.a[ 1 ] = temp_quaternion.y(); // save y axis
    this->vec_force_buoncy.a[ 2 ] = temp_quaternion.z(); // save z axis
    temp_quaternion = tf::Quaternion( 0, 0, -1.0*this->weight, 0 ); // Load weight force odom frame
    temp_quaternion = ptr_current_quaternion->inverse() * 
            temp_quaternion * 
            (*ptr_current_quaternion); // rotation force
    this->vec_force_gravity.a[ 0 ] = temp_quaternion.x(); // save x axis
    this->vec_force_gravity.a[ 1 ] = temp_quaternion.y(); // save y axis
    this->vec_force_gravity.a[ 2 ] = temp_quaternion.z(); // save z axis
    // Now we have linear force from buoncy force and weight
    // Next we will prepare matrix of buoncy & gravity force
    std::memcpy( (void*) &this->mat_force_buoncy.a[0][3],
            (void*) boost::qvm::cross( this->vec_force_buoncy, 
                    zeabus::robot::vec_center_buoncy ).a ,
            sizeof( double ) * 3 ); // save part moment force buoncy force 
    std::memcpy( (void*) this->mat_force_buoncy.a[0] , 
            (void*) this->vec_force_buoncy.a , 
            sizeof( double )* 3 ); // save part linear force by bouncy force
    std::memcpy( (void*) &this->mat_force_gravity.a[0][3] , 
            (void*)boost::qvm::cross( this->vec_force_gravity, 
                    zeabus::robot::vec_center_gravity).a ,
            sizeof( double ) * 3 ); // save part moment force by gravity force
    std::memcpy( (void*) this->mat_force_gravity.a[0] , 
            (void*)this->vec_force_gravity.a , 
            sizeof( double )* 3 ); // save part linear force by gravity force
#ifdef SHOW_DATA
    std::cout   << "BUONCY FORCE   :\n";
    zeabus_boost::print( this->mat_force_buoncy );
    std::cout   << "GRAVITY FORCE  :\n";
    zeabus_boost::print( this->mat_force_gravity );
#endif // SHOW_DATA

    // Next We will prepare matrix viscosity force
    this->mat_force_viscosty.a[ 0 ][ 0 ] = this->ptr_vec_current_velocity->a[ 0 ] *
            this->vec_constant_viscosty.a[ 0 ];
    this->mat_force_viscosty.a[ 0 ][ 1 ] = this->ptr_vec_current_velocity->a[ 1 ] *
            this->vec_constant_viscosty.a[ 1 ];
    this->mat_force_viscosty.a[ 0 ][ 2 ] = this->ptr_vec_current_velocity->a[ 2 ] *
            this->vec_constant_viscosty.a[ 2 ];
    this->mat_force_viscosty.a[ 0 ][ 3 ] = this->ptr_vec_current_velocity->a[ 3 ] *
            this->vec_constant_viscosty.a[ 3 ];
    this->mat_force_viscosty.a[ 0 ][ 4 ] = this->ptr_vec_current_velocity->a[ 4 ] *
            this->vec_constant_viscosty.a[ 4 ];
    this->mat_force_viscosty.a[ 0 ][ 5 ] = this->ptr_vec_current_velocity->a[ 5 ] *
            this->vec_constant_viscosty.a[ 5 ];
    this->mat_force_viscosty.a[ 0 ][ 6 ] = this->ptr_vec_current_velocity->a[ 6 ] *
            this->vec_constant_viscosty.a[ 6 ];
    this->mat_force_viscosty.a[ 0 ][ 7 ] = this->ptr_vec_current_velocity->a[ 7 ] *
            this->vec_constant_viscosty.a[ 7 ];
   // The last we must to prepare matrix robot force 
    this->mat_force_robot = ( *(this->ptr_mat_force_thruster ) ) * zeabus::robot::direction_all;

#ifdef SHOW_DATA
    std::cout   << "VISCOSTY FORCE :\n";
    zeabus_boost::print( this->mat_force_viscosty );
#endif // SHOW_DATA

#ifdef SHOW_ROBOT_FORCE
    std::cout   << "ROBOT FORCE    :\n";
    zeabus_boost::print( this->mat_force_robot );
#endif // SHOW_ROBOT_FORCE

    this->mat_force_sum = this->mat_force_viscosty +
            this->mat_force_robot +
            this->mat_force_gravity +
            this->mat_force_buoncy;

    this->vec_acceleration.a[0] = this->mat_force_sum.a[0][0] / this->mass;
    this->vec_acceleration.a[1] = this->mat_force_sum.a[0][1] / this->mass;
    this->vec_acceleration.a[2] = this->mat_force_sum.a[0][2] / this->mass;
    this->vec_acceleration.a[3] = this->mat_force_sum.a[0][3] / this->moment_inertia;
    this->vec_acceleration.a[4] = this->mat_force_sum.a[0][4] / this->moment_inertia;
    this->vec_acceleration.a[5] = this->mat_force_sum.a[0][5] / this->moment_inertia;

    *( this->ptr_vec_current_velocity ) += ( this->vec_acceleration * diff_time );

#ifdef SHOW_ROBOT_FORCE
    std::cout   << "ROBOT ACCEL    :\n";
    zeabus_boost::print( this->vec_acceleration );
    std::cout   << "NEW VELOCITY   :\n";
    zeabus_boost::print( *( this->ptr_vec_current_velocity ) );
#endif
} // function calculate
