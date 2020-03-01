// FILE			: updated_target_state.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, January 07 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <localize/updated_target_state.hpp>

UpdatedTargetState::UpdatedTargetState( ros::NodeHandle* ptr_node_handle )
{
    this->ptr_node_handle = ptr_node_handle;
}

void UpdatedTargetState::setup_all_variable( bool* ptr_updated_target_state,
        geometry_msgs::Pose* ptr_message_target_state,
        std::mutex* ptr_lock_target,
        geometry_msgs::Pose* ptr_message_current_state,
        std::mutex* ptr_lock_current )
{
    this->ptr_updated_target_state = ptr_updated_target_state;
    this->ptr_message_target_state = ptr_message_target_state;
    this->ptr_lock_target = ptr_lock_target;
    this->ptr_message_current_state = ptr_message_current_state;
    this->ptr_lock_current = ptr_lock_current;
}

void UpdatedTargetState::setup_subscriber( std::string topic_input , const double timeout )
{
    this->timeout = timeout;
    this->listener_mask_updated.setup_base( this->ptr_node_handle, &this->message );
    this->listener_mask_updated.setup_mutex_data( &this->lock_mask );
    this->listener_mask_updated.setup_subscriber_timestamp( topic_input , 1 );
}

void UpdatedTargetState::updated( ros::Time* time_stamp )
{
    bool updated = false;
    boost::array< bool , 6 > vector_mask;
    this->lock_mask.lock();
    if( ( *time_stamp - this->message.header.stamp).toSec() < this->timeout )
    {
        updated = true;
        vector_mask = this->message.mask;
    }
    this->lock_mask.unlock(); 

    if( updated )
    {
        // Start part updated orientation
        if( this->message.mask[ 3 ] || this->message.mask[ 4 ] || this->message.mask[ 5 ] )
        {
            tf::Quaternion quaternion;
            geometry_msgs::Vector3 current_vector;
            geometry_msgs::Vector3 target_vector;
            zeabus_ros::convert::geometry_quaternion::tf( 
                    &( this->ptr_message_current_state->orientation ),
                    &( quaternion ) );
            tf::Matrix3x3( quaternion ).getRPY( current_vector.x, 
                    current_vector.y,
                    current_vector.z );
            // Start lock target message
            this->ptr_lock_target->lock();
            zeabus_ros::convert::geometry_quaternion::tf(
                    &( this->ptr_message_target_state->orientation ),
                    &( quaternion ) );
            tf::Matrix3x3( quaternion ).getRPY( target_vector.x,
                    target_vector.y,
                    target_vector.z );
            if( vector_mask[ 3 ] ) target_vector.x = current_vector.x; 
            if( vector_mask[ 4 ] ) target_vector.y = current_vector.y; 
            if( vector_mask[ 5 ] ) target_vector.z = current_vector.z;
            quaternion.setRPY( target_vector.x , target_vector.y , target_vector.z ); 
            zeabus_ros::convert::geometry_quaternion::tf(
                    &( quaternion ),
                    &( this->ptr_message_target_state->orientation ) );
            this->ptr_lock_target->unlock();
        }

        // Start part updated position
        this->ptr_lock_target->lock();
        if( vector_mask[ 0 ] ) 
        {
            this->ptr_message_target_state->position.x = 
                    this->ptr_message_current_state->position.x;
        }
        if( vector_mask[ 1 ] ) 
        {
            this->ptr_message_target_state->position.y = 
                    this->ptr_message_current_state->position.y;
        }
        if( vector_mask[ 2 ] ) 
        {
            this->ptr_message_target_state->position.z = 
                    this->ptr_message_current_state->position.z;
        }
        *(this->ptr_updated_target_state) = true;
        this->ptr_lock_target->unlock();

    } // That mean your message will receive to reset target
}
