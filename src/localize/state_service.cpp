// FILE			: state_service.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 23 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <localize/state_service.hpp>

StateService::StateService( ros::NodeHandle* ptr_node_handle )
{
    this->ptr_node_handle = ptr_node_handle;
}

void StateService::setup_all_variable( nav_msgs::Odometry* ptr_message_current_state,
        std::mutex* ptr_lock_current )
{
    this->ptr_lock_current = ptr_lock_current;
    this->ptr_message_current_state = ptr_message_current_state; 
}

void StateService::setup_all_service()
{
    this->service_vision_state = this->ptr_node_handle->advertiseService( "/localize/vision"
            , &StateService::callback_vision_service , this );
}

bool StateService::callback_vision_service( zeabus_utility::StateConnection::Request& request,
        zeabus_utility::StateConnection::Response& response )
{
    if( request.header.frame_id == "odom" && request.child_frame_id == "base_link" )
    {
        printf( "Require update position xy from vision result\n");
        this->ptr_lock_current->lock();
        this->ptr_message_current_state->pose.pose.position.x = request.pose.x;
        this->ptr_message_current_state->pose.pose.position.y = request.pose.y;
        response.header = request.header;
        response.child_frame_id = request.child_frame_id;
        response.pose.z = this->ptr_message_current_state->pose.pose.position.z; 
        this->ptr_lock_current->unlock();
    }
    else
    {
        this->ptr_lock_current->lock();
        response.pose.x = this->ptr_message_current_state->pose.pose.position.x;
        response.pose.y = this->ptr_message_current_state->pose.pose.position.y;
        response.pose.z = this->ptr_message_current_state->pose.pose.position.z;
        response.header = this->ptr_message_current_state->header;
        response.child_frame_id = this->ptr_message_current_state->child_frame_id;
        this->ptr_lock_current->unlock();
    } 
}
