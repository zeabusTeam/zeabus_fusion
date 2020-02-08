// FILE			: calculate_by_force.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 08 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

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
//  And Other that is robot characteristic
//  Above requiredment make us know have 3 shared variable

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <zeabus/robot.hpp>

#include    <boost/qvm/vec_access.hpp>

#include    <boost/qvm/mat_access.hpp>

#include    <iostream>

class RobotForceHandle
{

    public:
        RobotForceHandle( const double mass , const double moment_inertia );

    protected:
        

};
