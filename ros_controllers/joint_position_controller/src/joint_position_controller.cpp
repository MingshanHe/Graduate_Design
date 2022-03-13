/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:09:41 
 * @Last Modified by: MingshanHe
 * @Last Modified time: 2021-12-05 04:18:32
 * @Licence: MIT Licence
 */
#include <pluginlib/class_list_macros.h>
#include "joint_position_controller/joint_position_controller.h"


namespace joint_position_controller
{
/** \brief Initialize the kinematic chain for kinematics-based computation.
 *
 */
bool Joint_Position_Controller::init(
    hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n) {

    // get publishing period
    if (!n.getParam("publish_rate", publish_rate_)){
        ROS_ERROR("Parameter 'publish_rate' not set");
        return false;
    }

    if (!n.getParam("joints", joints))
    {
        ROS_ERROR_STREAM("hardware can't get robot joints!");
        return false;
    }

    for (size_t i = 0; i < joints.size(); i++)
    {
        joint_handles_.push_back(robot->getHandle(joints[i]));
    }

    sub_command_ = n.subscribe("command_joint_pos", 5,
        &Joint_Position_Controller::command_joint_pos, this,
        ros::TransportHints().reliable().tcpNoDelay());

    joint_position_command.resize(4);
    joint_position_state.resize(4);
    joint_velocity_state.resize(4);
    joint_effort_state.resize(4);
    return true;
}


void Joint_Position_Controller::starting(const ros::Time& time){
    for(std::size_t i=0; i < 4; i++) {
        joint_position_state[i] = 0.0;
        joint_velocity_state[i] = 0.0;
        joint_effort_state[i] = 0.0;
    }
// //   End_Vel_Cmd_ = KDL::Twist::Zero();
    last_publish_time_ = time;
}

void Joint_Position_Controller::update(const ros::Time& time, const ros::Duration& period) {
    for(std::size_t i=0; i < this->joint_handles_.size(); i++)
    {
        joint_position_state[i] = joint_handles_[i].getPosition();
        joint_velocity_state[i] = joint_handles_[i].getVelocity();
        joint_effort_state[i] = joint_handles_[i].getEffort();
    }
//TODO: Add the joint trajectory.
    writePositionCommands(period);


    // Limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_
        + ros::Duration(1.0/publish_rate_) < time) {

        // try to publish
        // if (realtime_pub_->trylock()) {
        // last_publish_time_ = last_publish_time_
        //                     + ros::Duration(1.0/publish_rate_);
        // // populate message
        // // realtime_pub_->msg_.header.stamp = time;
        // // tf::poseKDLToMsg(End_Pos_, realtime_pub_->msg_.pose);
        // // tf::twistKDLToMsg(End_Vel_.GetTwist(), realtime_pub_->msg_.twist);

        // // realtime_pub_->unlockAndPublish();
        // }
    }
}

/*!
 * \brief Subscriber's callback: copies twist commands
 */
void Joint_Position_Controller::command_joint_pos(const joint_control_msg::JointControl &msg) {
    std::cout<<"In callback.----------------------------------------"<<std::endl;
    joint_position_command[0] = msg.joint1;
    joint_position_command[1] = msg.joint2;
    joint_position_command[2] = msg.joint3;
    joint_position_command[3] = msg.joint4;

}


/********************************************/
/**FUNCTIONS OF INSTANCES OF THE BASE CLASS**/
/********************************************/

/** \brief write the desired velocity command in the hardware interface input
 * for a VelocityJointInterface
 * \param period The duration of an update cycle
 */
void Joint_Position_Controller::writePositionCommands(
                                    const ros::Duration& period) {
    for(std::size_t i=0; i < this->joint_handles_.size(); i++) {
        this->joint_handles_[i].setCommand(joint_position_command[i]);
    }
}

} // controller_interface namespace

// Register controllers with the PLUGINLIB_EXPORT_CLASS macro to enable dynamic
// loading with the controller manager
PLUGINLIB_EXPORT_CLASS(joint_position_controller::Joint_Position_Controller, controller_interface::ControllerBase)
