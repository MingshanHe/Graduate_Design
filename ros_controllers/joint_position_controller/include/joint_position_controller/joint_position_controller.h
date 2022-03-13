/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:09:28 
 * @Last Modified by:   MingshanHe 
 * @Last Modified time: 2021-12-05 04:09:28 
 * @Licence: MIT Licence
 */
#ifndef JOINT_POSITION_CONTROLLER_H
#define JOINT_POSITION_CONTROLLER_H

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <vector>
#include "geometry_msgs/Quaternion.h"
#include "joint_control_msg/JointControl.h"
namespace joint_position_controller
{

/** \brief This class implements a ROS control cartesian velocity
 * controller. Its base class implements the core
 * of the controller.
 */
class Joint_Position_Controller: public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
    Joint_Position_Controller() {}
    ~Joint_Position_Controller() {}

    bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

    void command_joint_pos(const joint_control_msg::JointControl &msg);
private:

    void writePositionCommands(const ros::Duration& period);

protected:
    ros::Subscriber                                 sub_command_; // Interface to external commands

    ros::Time                                       last_publish_time_;
    double                                          publish_rate_;

    std::vector<hardware_interface::JointHandle>    joint_handles_;
    std::vector<std::string>                        joints;
    std::vector<double>                             joint_position_state;
    std::vector<double>                             joint_velocity_state;
    std::vector<double>                             joint_effort_state;
    std::vector<double>                             joint_position_command;
    // boost::shared_ptr<realtime_tools::RealtimePublisher<
    //     cartesian_state_msgs::PoseTwist> > realtime_pub_;
};
} // namespace controller_interface


#endif
