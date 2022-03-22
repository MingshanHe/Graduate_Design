#ifndef IMP_RBF_CONTROLLER_H
#define IMP_RBF_CONTROLLER_H

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <vector>
#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/src/>
#include <cmath>
#include <math.h>
#include "geometry_msgs/Quaternion.h"
namespace imp_rbf_controller
{

/** \brief This class implements a ROS control cartesian velocity
 * controller. Its base class implements the core
 * of the controller.
 */
class IMP_RBF_Controller: public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
    IMP_RBF_Controller() {}
    ~IMP_RBF_Controller() {}

    bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

    void command_joint_pos(const geometry_msgs::Quaternion &msg);

    void rbf_compute();

    void transform();

private:

    void writePositionCommands(const ros::Duration& period);

protected:
    ros::Subscriber                                 sub_command_; // Interface to external commands

    ros::Time                                       last_publish_time_;
    double                                          publish_rate_;

    std::vector<hardware_interface::JointHandle>    joint_handles_;
    std::vector<std::string>                        joints;
    Eigen::Matrix<double,4,1>                       joint_position_state;
    Eigen::Matrix<double,4,1>                       joint_velocity_state;
    Eigen::Matrix<double,4,1>                       joint_effort_state;
    std::vector<double>                             joint_position_command;
    
    Eigen::Matrix<double,4,1>                       position_error;
    Eigen::Matrix<double,4,1>                       velocity_error;
    Eigen::Matrix<double,4,1>                       tolext;

    double                                          dt;

    Eigen::Matrix<double,4,4>                       MSNN,CDNN;
    Eigen::Matrix<double,4,1>                       GSNN;
    Eigen::Matrix<double,4,4>                       M,D,K;

    /*------*/
    Eigen::Matrix<double, 4, 5> c_M;
    Eigen::Matrix<double, 4, 5> c_G;
    Eigen::Matrix<double, 8, 5> c_C;

    Eigen::Matrix<double, 8, 1> z;
    Eigen::Matrix<double, 8, 4> trans1;

    Eigen::Matrix<double, 5, 1> h_M11, h_M12, h_M13, h_M14, h_M21, h_M22, h_M23, h_M24, h_M31, h_M32, h_M33, h_M34, h_M41,h_M42,h_M43,h_M44;
    Eigen::Matrix<double, 5, 1> h_G1, h_G2, h_G3, h_G4;
    Eigen::Matrix<double, 5, 1> h_C11, h_C12, h_C13, h_C14, h_C21, h_C22, h_C23, h_C24, h_C31, h_C32, h_C33, h_C34, h_C41,h_C42,h_C43,h_C44;//hiden function
    
    Eigen::Matrix<double, 5, 1> W_M11, W_M12, W_M13, W_M14, W_M21, W_M22, W_M23, W_M24, W_M31, W_M32, W_M33, W_M34, W_M41,W_M42,W_M43,W_M44;
    Eigen::Matrix<double, 5, 1> W_G1, W_G2, W_G3, W_G4;
    Eigen::Matrix<double, 5, 1> W_C11, W_C12, W_C13, W_C14, W_C21, W_C22, W_C23, W_C24, W_C31, W_C32, W_C33, W_C34, W_C41,W_C42,W_C43,W_C44;//weight vector
    
    Eigen::Matrix<double, 4, 1> q, qd, e, d_q, d_e, d_qd, d_qr, dd_q, dd_qd, dd_qr;//r_��r��Ԫ��sign����ľ���
    Eigen::Matrix<double, 4, 1> r, r_,I;
    
    Eigen::Matrix<double, 5, 5> T_M11, T_M12, T_M13, T_M14, T_M21, T_M22, T_M23, T_M24, T_M31, T_M32, T_M33, T_M34, T_M41,T_M42,T_M43,T_M44;
    Eigen::Matrix<double, 5, 5> T_G1, T_G2, T_G3, T_G4;
    Eigen::Matrix<double, 5, 5> T_C11, T_C12, T_C13, T_C14, T_C21, T_C22, T_C23, T_C24, T_C31, T_C32, T_C33, T_C34, T_C41,T_C42,T_C43,T_C44;//eye vector
    
    Eigen::Matrix<double, 4, 4> fai,Hur;

    Eigen::Matrix<double, 4, 1> tolm, tolr, tol;
    Eigen::Matrix<double, 4, 4> Kp, Ki;

    double b;

    double Kr;

    double Mm,Gm,Cm;

    Eigen::Matrix<double,4,1> t;


    // std::vector<double>                             plus1;
    // std::vector<double>                             plus2;
    // std::vector<double>                             plus3;    

    // std::vector<double>                             error1;
    // std::vector<double>                             last_error1;
    // std::vector<double>                             next_error1;
    // std::vector<double>                             error2;
    // std::vector<double>                             last_error2;
    // std::vector<double>                             next_error2;
    // std::vector<double>                             error3;
    // std::vector<double>                             last_error3;
    // std::vector<double>                             next_error3;

    // double                                          Kp,Ki,Kd;

    // boost::shared_ptr<realtime_tools::RealtimePublisher<

    // boost::shared_ptr<realtime_tools::RealtimePublisher<
    // boost::shared_ptr<realtime_tools::RealtimePublisher<
    //     cartesian_state_msgs::PoseTwist> > realtime_pub_;
};
} // namespace controller_interface


#endif
