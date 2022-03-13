#include "device_hardware/device_hardware.h"
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <urdf/model.h>
#include <controller_manager/controller_manager.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hbrobot_control_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle node_root;

    std::string controllers_namespace, state_controller, position_controller;
    double control_frequency;


    if(!node_root.getParam("controllers_namespace", controllers_namespace))
    {
        ROS_ERROR_STREAM("ros con't find paramber \"controllers_namespace\"");
        return 0;
    }
    if(!node_root.getParam("position_controller", position_controller))
    {
        ROS_ERROR_STREAM("ros con't find paramber \"position_controller\"");
        return 0;
    }
    if(!node_root.getParam("state_controller", state_controller))
    {
        ROS_ERROR_STREAM("ros con't find paramber \"state_controller\"");
        return 0;
    }

    ros::NodeHandle node_robothw(node_root, controllers_namespace);

    device_hardware::device_hardware robot_hw;

    if (!robot_hw.init(node_root, node_robothw))
    {
        ROS_ERROR_STREAM("hardware init failure!");
        return 0;
    }
    controller_manager::ControllerManager cm(&robot_hw, node_root);

    //load controller
    if (!cm.loadController(controllers_namespace+"/"+position_controller))
    {
        ROS_ERROR_STREAM(position_controller<<" controller load failure!");
        return 0;
    }
    if (!cm.loadController(controllers_namespace+"/"+state_controller))
    {
        ROS_ERROR_STREAM(state_controller<<" controller load failure!");
        return 0;
    }

    // Set up timers
    ros::Time timestamp;
    ros::Duration period;
    auto stopwatch_last = std::chrono::steady_clock::now();
    auto stopwatch_now = stopwatch_last;

    double expected_cycle_time = 1.0 / (static_cast<double>(control_frequency));

    while (ros::ok())
    {

        robot_hw.read(timestamp, period);

        // Get current time and elapsed time since last read
        timestamp = ros::Time::now();
        stopwatch_now = std::chrono::steady_clock::now();
        period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
        stopwatch_last = stopwatch_now;

        cm.update(timestamp, period);

        robot_hw.write(timestamp, period);

        if (period.toSec() > expected_cycle_time)
        {
        // ROS_WARN_STREAM("Could not keep cycle rate of " << expected_cycle_time * 1000 << "ms");
        // ROS_WARN_STREAM("Actual cycle time:" << period.toNSec() / 1000000.0 << "ms");
        }

    }
    spinner.stop();
    ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");
    return 0;
}
