#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "/home/zby/exo1_ws/src/ros_controllers/impedance_RBF_controller/include/impedance_RBF_controller/impedance_RBF_controller.hpp"

#include <math.h>
#include <cmath>
#include <ros/ros.h>

namespace imp_rbf_controller
{
void IMP_RBF_Controller::transform()
{

for (std::size_t i=0;i<joint_handles_.size();i++)
{   
Eigen::Matrix<double,4,1> tr;
std::vector<double> t;
tr[i]=joint_handles_[i].getPosition();

}


}
}







#endif