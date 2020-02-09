// FILE			: mapper.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 07 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
#define SHOW_DATA

// MACRO CONDITION

#include    <calculate_by_force.hpp>

RobotForceHandle::RobotForceHandle( const double mass , 
        const double moment_inertia,
        const double volumn )
{
    this->weight = mass * zeabus::robot::gravity;
    this->buoncy = zeabus::robot::rho_water * zeabus::robot::gravity * volumn;
    this->mass = mass;
    this->moment_inertia = moment_inertia; 
}

void RobotForceHandle::setup_ptr_data( tf::Quaternion* ptr_current_quaternion ,
        boost::qvm::mat< double , 1 , 8 >* ptr_mat_force_thruster,
        boost::qvm::vec< double , 6>* ptr_vec_current_velocity )
{
    this->ptr_current_quaternion = ptr_current_quaternion;
    this->ptr_mat_force_thruster = ptr_mat_force_thruster;
    this->ptr_vec_current_velocity = ptr_vec_current_velocity;
}

void RobotForceHandle::calculate()
{
#ifdef SHOW_DATA
    std::cout   << "CURRENT_FORCE  :\n\t" ;
    zeabus_boost::print( *( this->ptr_mat_force_thruster ) );
#endif // SHOW_DATA
}
