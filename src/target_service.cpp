// FILE			: target_service.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, October 30 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

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
        std::mutex* ptr_lock )
{
    this->ptr_updated_target_state = ptr_update_target_state;
    this->ptr_message_target_state = ptr_message_target_state;
    this->ptr_lock = ptr_lock;
}

void TargetService::setup_all_service()
{
    this->service_absolute_depth = this->ptr_node_handle->advertiseService( "/target/absolute_depth"
            , &TargetService::callback_absolute_depth , this );
}

bool TargetService::callback_absolute_depth( zeabus_utility::SendFloat::Request& request,
        zeabus_utility::SendFloat::Response& response )
{
    this->ptr_lock->lock();
    this->ptr_message_target_state->position.z = request.data;
    *(this->ptr_updated_target_state) = true;
    this->ptr_lock->unlock();
}

#endif
