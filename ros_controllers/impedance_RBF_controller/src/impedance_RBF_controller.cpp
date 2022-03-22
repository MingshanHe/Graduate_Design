#include <pluginlib/class_list_macros.h>
#include "/home/zby/exo1_ws/src/ros_controllers/impedance_RBF_controller/include/impedance_RBF_controller/impedance_RBF_controller.hpp"
#include "/home/zby/exo1_ws/src/ros_controllers/impedance_RBF_controller/include/impedance_RBF_controller/rbf_compute.h"


namespace imp_rbf_controller
{
/** \brief Initialize the kinematic chain for kinematics-based computation.
 *
 */
bool IMP_RBF_Controller::init(
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
        &IMP_RBF_Controller::command_joint_pos, this,
        ros::TransportHints().reliable().tcpNoDelay());

    joint_position_command.resize(4);
    joint_position_state.resize(4);
    joint_velocity_state.resize(4);
    joint_effort_state.resize(4);
    return true;
}


void IMP_RBF_Controller::starting(const ros::Time& time){
    for(std::size_t i=0; i < 4; i++) {
        joint_position_state[i] = 0.0;
        joint_velocity_state[i] = 0.0;
        joint_effort_state[i] = 0.0;

        position_error[i]=0.0;
        velocity_error[i]=0.0;



        // plus1[i] = 0.0;
        // plus2[i] = 0.0;
        // plus3[i] = 0.0;
        // error1[i] = 0.0;
        // error2[i] = 0.0;
        // error3[i] = 0.0;
        // next_error1[i] = 0.0;
        // next_error2[i] = 0.0;
        // next_error3[i] = 0.0;
        // last_error1[i] = 0.0;
        // last_error2[i] = 0.0;
        // last_error3[i] = 0.0;
        // //casual parameters, we need to try to get a better set of PID parameters
        // YAML::Node pid_param = YAML::LoadFile("/home/zby/exo_ws/src/ros_controllers/pid_controller/config/pid_param.yaml");
        // Kp = pid_param["Kp"].as<double>();
        // Ki = pid_param["Ki"].as<double>();
        // Kd = pid_param["Kd"].as<double>();

        // Kp = 0.1;
        // Kd = 0.1;
        // Ki = 0.1;
    }
// //   End_Vel_Cmd_ = KDL::Twist::Zero();
    last_publish_time_ = time;
}

void IMP_RBF_Controller::update(const ros::Time& time, const ros::Duration& period) {
    for(std::size_t i=0; i < this->joint_handles_.size(); i++)
    {
        position_error[i]=joint_handles_[i].getPosition()-joint_position_state[i];
        velocity_error[i]=joint_handles_[i].getVelocity()-joint_velocity_state[i];
        
        rbf_compute();
      
        double a=joint_handles_[i].getVelocity();
        tolext=M*velocity_error/dt+D*velocity_error+K*position_error;

        
        t[i]=joint_handles_[i].getVelocity();

        

        //如何将joint_handles_[i].转换成Matrix形式

        // error1[i] = joint_handles_[i].getPosition()-joint_position_state[i];
        // plus1[i] = Kp*(error1[i]-next_error1[i])+Ki*error1[i]+Kd*(error1[i]-2*next_error1[i]+last_error1[i]);
        // joint_position_state[i] += plus1[i];
        // last_error1[i]=next_error1[i];
        // next_error1[i]=error1[i]; 

        // error2[i] = joint_handles_[i].getVelocity()-joint_velocity_state[i];
        // plus2[i] = Kp*(error2[i]-next_error2[i])+Ki*error2[i]+Kd*(error2[i]-2*next_error2[i]+last_error2[i]);
        // joint_velocity_state[i+1] += plus2[i];
        // last_error2[i]=next_error2[i];
        // next_error2[i]=error2[i]; 

        // error3[i] = joint_handles_[i].getEffort()-joint_effort_state[i];
        // plus3[i] = Kp*(error3[i]-next_error3[i])+Ki*error3[i]+Kd*(error3[i]-2*next_error3[i]+last_error3[i]);
        // joint_effort_state[i] += plus3[i];
        // last_error3[i]=next_error3[i];
        // next_error3[i]=error3[i]; 

        // joint_position_state[i] = joint_handles_[i].getPosition();
        // joint_velocity_state[i] = joint_handles_[i].getVelocity();
        // joint_effort_state[i] = joint_handles_[i].getEffort();
    }
    dt=0.01;
    joint_effort_state=MSNN*t/dt+CDNN*t+GSNN+MSNN*M.inverse()*(D*velocity_error+K*position_error)+(tolext-MSNN*M.inverse()*tolext);

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
void IMP_RBF_Controller::command_joint_pos(const geometry_msgs::Quaternion &msg) {
    joint_position_command[0] = msg.x;
    joint_position_command[1] = msg.y;
    joint_position_command[2] = msg.z;
    joint_position_command[3] = msg.w;

}


/********************************************/
/**FUNCTIONS OF INSTANCES OF THE BASE CLASS**/
/********************************************/

/** \brief write the desired velocity command in the hardware interface input
 * for a VelocityJointInterface
 * \param period The duration of an update cycle
 */
void IMP_RBF_Controller::writePositionCommands(
                                    const ros::Duration& period) {
    for(std::size_t i=0; i < this->joint_handles_.size(); i++) {
      this->joint_handles_[i].setCommand(joint_position_command[i]);
    }
}

} // controller_interface namespace

// Register controllers with the PLUGINLIB_EXPORT_CLASS macro to enable dynamic
// loading with the controller manager
PLUGINLIB_EXPORT_CLASS(imp_rbf_controller::IMP_RBF_Controller, controller_interface::ControllerBase)
