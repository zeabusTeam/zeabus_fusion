// FILE			: mapper_reconfigure.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 10 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <dynamic_reconfigure/server.h>

#include    <zeabus_localize/VisualConfig.h>

#include    <boost/qvm/vec_access.hpp>

#include    <mapper/calculate_by_force.hpp>

#include    <mutex>

#ifndef _ZEABUS_LOCALIZE_MAPPER_RECONFIGURE_HPP__
#define _ZEABUS_LOCALIZE_MAPPER_RECONFIGURE_HPP__

void dynamic_reconfigure_callback( zeabus_localize::VisualConfig &config , uint32_t level );

void dynamic_reconfigure_set_parameter( RobotForceHandle* ch );

#endif // _ZEABUS_LOCALIZE_MAPPER_RECONFIGURE_HPP__
