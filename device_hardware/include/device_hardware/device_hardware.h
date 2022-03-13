#ifndef DEVICE_HARDWARE_H
#define DEVICE_HARDWARE_H

#include "device_driver/device_driver.h"
// #include "device_driver/eci/EciDemo113.h"
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <math.h>
// #include <unistd.h>
// #include <errno.h>
// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
// #include <termios.h>

// #include <vector>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <pluginlib/class_list_macros.h>
namespace device_hardware
{


    struct Robot_Information
    {

        std::vector<std::string> joints;
        size_t joints_num;

        std::vector<double> joint_position_state_;
        std::vector<double> joint_velocity_state_;
        std::vector<double> joint_effort_state_;

        std::vector<double> joint_position_command_;
        std::vector<double> joint_velocity_command_;
        std::vector<double> joint_effort_command_;

        std::vector<std::string> joint_name_;

        std::vector< hardware_interface::JointHandle >  pj_handle_;
        std::vector< hardware_interface::JointHandle >  vj_handle_;
        std::vector< hardware_interface::JointHandle >  ej_handle_;

    };

    class device_hardware : public hardware_interface::RobotHW
    {
    private:
        /* data */
    public:
        device_hardware(/* args */){};

        virtual ~device_hardware(){};

        virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);

        virtual void read(const ros::Time& time, const ros::Duration& period);

        virtual void write(const ros::Time& time, const ros::Duration& period);

        // virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
        //                         const std::list<hardware_interface::ControllerInfo>& stop_list);

        // virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
        //                     const std::list<hardware_interface::ControllerInfo>& stop_list);

    private:

        Robot_Information                           Robot;
        // hardware_interface
        hardware_interface::JointStateInterface     js_interface_;

        hardware_interface::PositionJointInterface  pj_interface_;
        hardware_interface::VelocityJointInterface  vj_interface_;
        hardware_interface::EffortJointInterface    ej_interface_;


        //joint_limits_interface


        // joint_limits_interface::PositionJointSaturationInterface  pj_limits_interface_;
        // joint_limits_interface::PositionJointSoftLimitsInterface  pj_soft_limits_interface_;
        // joint_limits_interface::VelocityJointSaturationInterface  vj_limits_interface_;
        // joint_limits_interface::VelocityJointSoftLimitsInterface  vj_soft_limits_interface_;
        // joint_limits_interface::EffortJointSaturationInterface    ej_limits_interface_;
        // joint_limits_interface::EffortJointSoftLimitsInterface    ej_soft_limits_interface_;

        // typedef boost::shared_ptr<const urdf::Joint>   JointConstSharedPtr;
        //std::vector<JointConstSharedPtr> joint_urdfs_;
        // std::vector<urdf::JointConstSharedPtr> joint_urdfs_; //kinetic-devel

        std::string commond_controller;
        std::string hyy_controller;
        // int controller_flag;
        //-----------------------------------------------------------------------------
        // bool _sim_flag;

        // double joint_state_test[2][7];
        // double joint_state_command_[2][7];

        std::vector<double> position_tmp;
    };

}

#endif