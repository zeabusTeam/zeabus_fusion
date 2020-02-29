// FILE			: target_service.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, October 30 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  Relative callback will relative with target message 

// REFERENCE
// ref01 : http://docs.ros.org/melodic/api/tf/html/c++/Quaternion_8h_source.html#l00076

// MACRO SET

// MACRO CONDITION

#include    <target_service.hpp>

#ifdef _TARGET_SERVICE_HPP__

TargetService::TargetService( ros::NodeHandle* ptr_node_handle )
{
    this->ptr_node_handle = ptr_node_handle;
}

void TargetService::setup_all_variable( bool* ptr_update_target_state,
        geometry_msgs::Pose* ptr_message_target_state,
        std::mutex* ptr_lock_target,
        geometry_msgs::Pose* ptr_message_current_state,
        std::mutex* ptr_lock_current )
{
    this->ptr_updated_target_state = ptr_update_target_state;
    this->ptr_message_target_state = ptr_message_target_state;
    this->ptr_message_current_state = ptr_message_current_state;
    this->ptr_lock_current = ptr_lock_current;
    this->ptr_lock_target = ptr_lock_target;
}

void TargetService::setup_all_service()
{

    this->service_absolute_depth = this->ptr_node_handle->advertiseService( "/target/absolute/depth"
            , &TargetService::callback_absolute_depth , this );
    this->service_relative_depth = this->ptr_node_handle->advertiseService( "/target/relative/depth"
            , &TargetService::callback_relative_depth , this );
    this->service_relative_roll = this->ptr_node_handle->advertiseService( "/target/relative/roll"
            , &TargetService::callback_relative_roll , this );
    this->service_relative_pitch = this->ptr_node_handle->advertiseService( "/target/relative/pitch"
            , &TargetService::callback_relative_pitch , this );
    this->service_relative_yaw = this->ptr_node_handle->advertiseService( "/target/relative/yaw"
            , &TargetService::callback_relative_yaw , this );
    this->service_absolute_roll = this->ptr_node_handle->advertiseService( "/target/absolute/roll"
            , &TargetService::callback_absolute_roll , this );
    this->service_absolute_pitch = this->ptr_node_handle->advertiseService( "/target/absolute/pitch"
            , &TargetService::callback_absolute_pitch , this );
    this->service_absolute_yaw = this->ptr_node_handle->advertiseService( "/target/absolute/yaw"
            , &TargetService::callback_absolute_yaw , this );
    this->service_reset_all = this->ptr_node_handle->advertiseService( "/target/reset/all"
            , &TargetService::callback_reset_all , this );
}

bool TargetService::callback_absolute_depth( zeabus_utility::SendFloat::Request& request,
        zeabus_utility::SendFloat::Response& response )
{
    this->ptr_lock_target->lock(); // acquire lock
    this->ptr_message_target_state->position.z = request.data;
    this->updated_target_state();
    this->ptr_lock_target->unlock(); // release lock
    return true;
}

bool TargetService::callback_relative_depth( zeabus_utility::SendFloat::Request& request, 
        zeabus_utility::SendFloat::Response& reponse )
{
    this->ptr_lock_target->lock(); // acquire lock
    this->ptr_message_target_state->position.z += request.data;
    this->updated_target_state();
    this->ptr_lock_target->unlock(); // release lock
    return true;
}

// Part of relative orientation
bool TargetService::callback_relative_roll( zeabus_utility::SendFloat::Request& request,
        zeabus_utility::SendFloat::Response& response )
{
    tf::Quaternion relative_quaternion;
    tf::Quaternion current_quaternion; 
    zeabus_ros::convert::geometry_quaternion::tf( &(this->ptr_message_target_state->orientation) ,
            &current_quaternion );
    relative_quaternion.setRPY( request.data , 0 , 0 );
    relative_quaternion = current_quaternion * relative_quaternion * current_quaternion.inverse();
    current_quaternion = relative_quaternion * current_quaternion;
    this->ptr_lock_target->lock(); // acquire lock
    zeabus_ros::convert::geometry_quaternion::tf( &current_quaternion , 
            &( this->ptr_message_target_state->orientation ) );
    this->updated_target_state();
    this->ptr_lock_target->unlock(); // release lock
    return true;
}

bool TargetService::callback_relative_pitch( zeabus_utility::SendFloat::Request& request,
        zeabus_utility::SendFloat::Response& response )
{
    tf::Quaternion relative_quaternion;
    tf::Quaternion current_quaternion; 
    zeabus_ros::convert::geometry_quaternion::tf( &(this->ptr_message_target_state->orientation) ,
            &current_quaternion );
    relative_quaternion.setRPY( 0 , request.data , 0 );
    relative_quaternion = current_quaternion * relative_quaternion * current_quaternion.inverse();
    current_quaternion = relative_quaternion * current_quaternion;
    this->ptr_lock_target->lock(); // acquire lock
    zeabus_ros::convert::geometry_quaternion::tf( &current_quaternion , 
            &( this->ptr_message_target_state->orientation ) );
    this->updated_target_state();
    this->ptr_lock_target->unlock(); // release lock
    return true;
}

bool TargetService::callback_relative_yaw( zeabus_utility::SendFloat::Request& request,
        zeabus_utility::SendFloat::Response& response )
{
    tf::Quaternion relative_quaternion;
    tf::Quaternion current_quaternion; 
    zeabus_ros::convert::geometry_quaternion::tf( &(this->ptr_message_target_state->orientation) ,
            &current_quaternion );
    relative_quaternion.setRPY( 0 , 0 , request.data );
    relative_quaternion = current_quaternion * relative_quaternion * current_quaternion.inverse();
    current_quaternion = relative_quaternion * current_quaternion;
    this->ptr_lock_target->lock(); // acquire lock
    zeabus_ros::convert::geometry_quaternion::tf( &current_quaternion , 
            &( this->ptr_message_target_state->orientation ) );
    this->updated_target_state();
    this->ptr_lock_target->unlock(); // release lock
    return true;
}
// part of absolute orientation
bool TargetService::callback_absolute_roll( zeabus_utility::SendFloat::Request& request,
        zeabus_utility::SendFloat::Response& response )
{
    geometry_msgs::Vector3 temp_vector3;
    tf::Quaternion current_quaternion; 
    zeabus_ros::convert::geometry_quaternion::tf( &(this->ptr_message_target_state->orientation) ,
            &current_quaternion );
    tf::Matrix3x3( current_quaternion ).getRPY( temp_vector3.x , temp_vector3.y , temp_vector3.z );
    current_quaternion.setRPY( request.data , temp_vector3.y , temp_vector3.z );
    this->ptr_lock_target->lock(); // acquire lock
    zeabus_ros::convert::geometry_quaternion::tf( &current_quaternion , 
            &( this->ptr_message_target_state->orientation ) );
    this->updated_target_state();
    this->ptr_lock_target->unlock(); // release lock
    return true;
}

bool TargetService::callback_absolute_pitch( zeabus_utility::SendFloat::Request& request,
        zeabus_utility::SendFloat::Response& response )
{
    geometry_msgs::Vector3 temp_vector3;
    tf::Quaternion current_quaternion; 
    zeabus_ros::convert::geometry_quaternion::tf( &(this->ptr_message_target_state->orientation) ,
            &current_quaternion );
    tf::Matrix3x3( current_quaternion ).getRPY( temp_vector3.x , temp_vector3.y , temp_vector3.z );
    current_quaternion.setRPY( temp_vector3.x , request.data , temp_vector3.z );
    this->ptr_lock_target->lock(); // acquire lock
    zeabus_ros::convert::geometry_quaternion::tf( &current_quaternion , 
            &( this->ptr_message_target_state->orientation ) );
    this->updated_target_state();
    this->ptr_lock_target->unlock(); // release lock
    return true;
}

bool TargetService::callback_absolute_yaw( zeabus_utility::SendFloat::Request& request,
        zeabus_utility::SendFloat::Response& response )
{
    geometry_msgs::Vector3 temp_vector3;
    tf::Quaternion current_quaternion; 
    zeabus_ros::convert::geometry_quaternion::tf( &(this->ptr_message_target_state->orientation) ,
            &current_quaternion );
    tf::Matrix3x3( current_quaternion ).getRPY( temp_vector3.x , temp_vector3.y , temp_vector3.z );
    current_quaternion.setRPY( temp_vector3.x , temp_vector3.z , request.data );
    this->ptr_lock_target->lock(); // acquire lock
    zeabus_ros::convert::geometry_quaternion::tf( &current_quaternion , 
            &( this->ptr_message_target_state->orientation ) );
    this->updated_target_state();
    this->ptr_lock_target->unlock(); // release lock
    return true;
}

// Part reset target state
bool TargetService::callback_reset_all( zeabus_utility::SendBool::Request& request,
        zeabus_utility::SendBool::Response& reponse )
{
    if( request.data )
    {
        this->ptr_lock_target->lock();
        this->ptr_lock_current->lock();
        *(this->ptr_message_target_state) = *(this->ptr_message_current_state );
        this->updated_target_state();
        this->ptr_lock_target->unlock();
        this->ptr_lock_current->unlock();
    }
    else
    {
        this->ptr_lock_target->lock();
        this->ptr_lock_current->lock();
        this->ptr_message_target_state->orientation = this->ptr_message_current_state->orientation;
        this->updated_target_state();
        this->ptr_lock_target->unlock();
        this->ptr_lock_current->unlock();
    }
    return true;
}

void TargetService::updated_target_state()
{
    *(this->ptr_updated_target_state) = true;
}
#endif
