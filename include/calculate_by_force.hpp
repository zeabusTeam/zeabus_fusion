// FILE			: calculate_by_force.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 08 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
//  SHOW_DATA   : This will show all daa vector and matrix when use

// README
//  This module will responsible to calculate current velocity in robot frame to you
//  So we must can access some data to use to calculate.
//  Data required
//      - current quaternion
//      - current velocity
//      - current force
//  And static value must to know
//      - mass
//      - moment_inertia
//      - constant of viscouty
//      - volumn
//  And Other that is robot characteristic
//  Above requiredment make us know have 3 shared variable

// REFERENCE

// MACRO SET
//#define SHOW_DATA

// MACRO CONDITION

#include    <zeabus/robot.hpp>

#include    <boost/qvm/vec_access.hpp>

#include    <boost/qvm/mat_access.hpp>

#include    <tf/LinearMath/Quaternion.h>

#include    <zeabus/math/boost/print_data.hpp>

#include    <cstring> // include function memcpy

#include    <iostream>

class RobotForceHandle
{

    public:
        RobotForceHandle( const double mass , const double moment_inertia , const double volumn );

        void setup_ptr_data( tf::Quaternion* ptr_current_quaternion ,
                boost::qvm::mat< double , 1 , 8 >* ptr_mat_force_thruster,
                boost::qvm::vec< double , 6 >* ptr_vec_current_velocity );

        void calculate();

    protected:
        double weight;
        double buoncy;
        double mass;
        double moment_inertia;
        // shared data
        tf::Quaternion* ptr_current_quaternion;
        boost::qvm::vec< double , 6 >* ptr_vec_current_velocity;
        boost::qvm::mat< double , 1 , 8 >* ptr_mat_force_thruster;

        boost::qvm::mat< double , 1 , 6 > mat_force_robot;
        boost::qvm::mat< double , 1 , 6 > mat_force_viscosty;
        boost::qvm::mat< double , 1 , 6 > mat_force_gravity;
        boost::qvm::mat< double , 1 , 6 > mat_force_buoncy;
        boost::qvm::mat< double , 1 , 6 > mat_force_sum;

        boost::qvm::vec< double , 3 > vec_force_gravity;
        boost::qvm::vec< double , 3 > vec_force_bunocy;
        boost::qvm::vec< double , 6 > vec_constant_viscosty;
        boost::qvm::vec< double , 6 > vec_acceleration;        


};
