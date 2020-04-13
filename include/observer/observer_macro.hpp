// FILE			: observer_macro.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, ????? ?? (UTC+0)
// MAINTAINER	: K.Supasan

// ==================> MACRO DETAIL

// ==================> README

// ==================> REFERENCE

// ==================> MACRO SET
//#define _USE_VISCOSITY_LINEAR_
#define _USE_VISCOSITY_EXPONENTIAL_

//#define _RECEIVE_VISION_DATA_

//#define _PRINT_PROCESS_CALCULATE_MODEL_

// ==================> MACRO CONDITION
// Part Manage about how to calculate viscosity
#ifdef _USE_VISCOSITY_LINEAR_
    #undef _USE_VISCOSITY_EXPONENTIAL_
#endif
#ifdef _USE_VISCOSITY_EXPONENTIAL_
    #undef _USE_VISCOSITY_LINEAR_
#endif
#ifndef _USE_VISCOSITY_EXPONENTIAL_
#ifndef _USE_VISCOSITY_LINEAR_
    #define _USE_VISCOSITY_EXPONENTIAL_
#endif // _USE_VISCOSITY_LINEAR_
#endif // _USE_VISCOSITY_EXPONENTIAL_
