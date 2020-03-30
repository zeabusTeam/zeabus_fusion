// FILE			: observer_model.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 10 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

extern boost::qvm::mat< double , 6 , 1 > mat_acceleration;
extern boost::qvm::mat< double , 6 , 1 > mat_force_gravity;
extern boost::qvm::mat< double , 6 , 1 > mat_force_buoncy;
extern boost::qvm::mat< double , 6 , 1 > mat_force_constant;
extern boost::qvm::mat< double , 6 , 1 > mat_force_viscosity;
extern double roll;
extern double pitch;
extern double yaw;

struct DataObserverModel
{
    boost::qvm::mat< double , 6 , 1 > mat_acceleration; // output predict use to find cost value
    boost::qvm::mat< double , 6 , 1 > mat_force_gravity; // gravity force after collect
    boost::qvm::mat< double , 6 , 1 > mat_force_buoncy; // buoncy force after collect
    boost::qvm::mat< double , 6 , 1 > mat_force_constant; // const force not pamameter to clibration
    boost::qvm::mat< double , 6 , 1 > mat_force_thruster; // Force thruster
    // Above 5 variable for is base link not want to ratation again
    // About velocity for finding viscosity force
    //      In case config by vision data : will use velocity from vision
    //      In case rotation and localize waiiting process
    tf::Quaternion quaternion;
    ros::Time stamp;
    boost::array< double , 6 > arr_robot_velocity;

    DataObserverModel( const boost::qvm::mat< double , 6 , 1 >& mat_acceleration,
            const boost::qvm::mat< double , 6 , 1 >& mat_force_gravity, 
            const boost::qvm::mat< double , 6 , 1 >& mat_force_buoncy,
            const boost::qvm::mat< double , 6 , 1 >& mat_force_constant, 
            const boost::qvm::mat< double , 6 , 1 >& mat_force_thruster,
            const boost::array< double , 6 >& arr_robot_velocity,
            const tf::Quaternion& quaternion,
            const ros::Time& stamp )
    {
        this->mat_acceleration = mat_acceleration;
        this->mat_force_thruster = mat_force_thruster;
        this->mat_force_gravity = mat_force_gravity;
        this->mat_force_buoncy = mat_force_buoncy;
        this->mat_force_constant = mat_force_constant;
        this->arr_robot_velocity = arr_robot_velocity;
        this->stamp = stamp;
        this->quaternion = quaternion;
    }

    DataObserverModel(){};
}; 
extern std::vector< DataObserverModel > vec_model_data;

#ifndef _CPE_PROJECT_ZEABUS_LOCALIZE_OBSERVER_OBSERVER_MODEL_HPP__
#define _CPE_PROJECT_ZEABUS_LOCALIZE_OBSERVER_OBSERVER_MODEL_HPP__
void report_model();
void active_model( const ros::Time& current_time );
void calculate_viscosity();
inline double viscosity( const unsigned int index );
void check_buffer_model( const ros::Time& minimum_time );
void reset_buffer_model();
bool search_buffer_model( const ros::Time& target_time , DataObserverModel* ptr_data );
// return true in case have data in buffer
#endif // _CPE_PROJECT_ZEABUS_LOCALIZE_OBSERVER_OBSERVER_MODEL_HPP__
