// FILE			: know_robot.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, January 18 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
//  ref01 : http://www.cplusplus.com/reference/iomanip/setprecision/
//  ref02 : https://boostorg.github.io/qvm/#vec_access

// MACRO SET

// MACRO CONDITION

#include    <zeabus/robot.hpp>

#include    <iostream>

#include    <boost/qvm/vec_access.hpp>

#include    <boost/qvm/mat_access.hpp>

#include    <iomanip>

int main( int argv , char** argc )
{

    std::cout   << std::setprecision( 3 ) << std::setw( 7 );
    
    std::cout   << "Direction of linear force is : \n";
    for( unsigned int run = 0 ; run < 8 ; run++ )
    {
        std::cout   << "Thruster number " << run  << " :";
        std::cout   << std::setprecision( 3 ) << std::setw( 8 )
                    << boost::qvm::A0( zeabus::robot::direction_linear_force[ run ] )
                    << std::setprecision( 3 ) << std::setw( 8 )
                    << boost::qvm::A1( zeabus::robot::direction_linear_force[ run ] )
                    << std::setprecision( 3 ) << std::setw( 8 )
                    << boost::qvm::A2( zeabus::robot::direction_linear_force[ run ] )
                    << "\n";
    }

    std::cout   << "\nDirection of moment torque is : \n";
    for( unsigned int run = 0 ; run < 8 ; run++ )
    {
        std::cout   << "Thruster number " << run  << " :";
        std::cout   << std::setprecision( 3 ) << std::setw( 8 )
                    << boost::qvm::A0( zeabus::robot::direction_moment[ run ] )
                    << std::setprecision( 3 ) << std::setw( 8 )
                    << boost::qvm::A1( zeabus::robot::direction_moment[ run ] )
                    << std::setprecision( 3 ) << std::setw( 8 )
                    << boost::qvm::A2( zeabus::robot::direction_moment[ run ] )
                    << "\n";
    }

    std::cout   << "\nMatrix for direction force and torque is : \n";
    std::cout   << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A00( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A01( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A02( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A03( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A04( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A05( zeabus::robot::direction_all ) << "\n"
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A10( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A11( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A12( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A13( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A14( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A15( zeabus::robot::direction_all ) << "\n"
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A20( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A21( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A22( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A23( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A24( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A25( zeabus::robot::direction_all ) << "\n"
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A30( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A31( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A32( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A33( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A34( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A35( zeabus::robot::direction_all ) << "\n"
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A40( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A41( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A42( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A43( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A44( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A45( zeabus::robot::direction_all ) << "\n"
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A50( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A51( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A52( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A53( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A54( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A55( zeabus::robot::direction_all ) << "\n"
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A60( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A61( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A62( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A63( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A64( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A65( zeabus::robot::direction_all ) << "\n"
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A70( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A71( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A72( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A73( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A74( zeabus::robot::direction_all )
                << std::setprecision( 3 ) << std::setw( 8 )
                << boost::qvm::A75( zeabus::robot::direction_all ) << "\n";

}
