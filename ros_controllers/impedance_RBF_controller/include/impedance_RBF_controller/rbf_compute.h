#ifndef RBF_COMPUTE_H
#define RBF_COMPUTE_H

#include "/home/zby/exo1_ws/src/ros_controllers/impedance_RBF_controller/include/impedance_RBF_controller/impedance_RBF_controller.hpp"
#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include "yaml-cpp/yaml.h"

namespace imp_rbf_controller
{   
    void IMP_RBF_Controller::rbf_compute()
    {
    for(std::size_t i=0; i < this->joint_handles_.size(); i++)
        {
            double dt = 0.01;
             q[i] = joint_handles_[i].getPosition();
             qd[i] = joint_position_state[i];
             d_q[i] = joint_handles_[i].getVelocity();
             d_qd[i] = joint_velocity_state[i];
             dd_qd[i] = joint_velocity_state[i]/dt;
             
        }
    YAML::Node imp_rbf_param = YAML::LoadFile("/home/zby/exo1_ws/src/ros_controllers/impedance_RBF_controller/config/imp_RBF.yaml");
    // c_M << imp_rbf_param["c_M"].as<Eigen::Matrix<double,4,5>>();
    // c_G << imp_rbf_param["c_G"].as<Eigen::Matrix<double,4,5>>();
    // c_C << imp_rbf_param["c_C"].as<Eigen::Matrix<double,8,5>>();
 
    
    c_M << -1, -0.5, 0, 0.5, 1,
        -1, -0.5, 0, 0.5, 1,
        -1, -0.5, 0, 0.5, 1,
        -1, -0.5, 0, 0.5, 1;
    c_G << -1, -0.5, 0, 0.5, 1,
        -1, -0.5, 0, 0.5, 1,
        -1, -0.5, 0, 0.5, 1,
        -1, -0.5, 0, 0.5, 1;
    c_C << -1, -0.5, 0, 0.5, 1,
        -1, -0.5, 0, 0.5, 1,
        -1, -0.5, 0, 0.5, 1,
        -1, -0.5, 0, 0.5, 1,
        -1, -0.5, 0, 0.5, 1,
        -1, -0.5, 0, 0.5, 1,
        -1, -0.5, 0, 0.5, 1,
        -1, -0.5, 0, 0.5, 1;

    // T_M11 << imp_rbf_param["T_M11"].as<Eigen::Matrix<double,5,5>>();
    T_M11 << 5, 0, 0, 0, 0,
        0, 5, 0, 0, 0,
        0, 0, 5, 0, 0,
        0, 0, 0, 5, 0,
        0, 0, 0, 0, 5;
    T_M12=T_M11;T_M13=T_M11;T_M14=T_M11;T_M21=T_M11;T_M22=T_M11;T_M23=T_M11;T_M24=T_M11;
    T_M31=T_M11;T_M32=T_M11;T_M33=T_M11;T_M34=T_M11;T_M41=T_M11;T_M42=T_M11;T_M43=T_M11;T_M44=T_M11;


    // T_G1 << imp_rbf_param["T_G1"].as<Eigen::Matrix<double,5,5>>();
    T_G1<< 10, 0, 0, 0, 0,
        0, 10, 0, 0, 0,
        0, 0, 10, 0, 0,
        0, 0, 0, 10, 0,
        0, 0, 0, 0, 10;
    T_G2=T_G1; T_G3=T_G1;T_G4=T_G1;

    // T_C11 << imp_rbf_param["T_C11"].as<Eigen::Matrix<double,5,5>>();
    T_C11 << 10, 0, 0, 0, 0,
        0, 10, 0, 0, 0,
        0, 0, 10, 0, 0,
        0, 0, 0, 10, 0,
        0, 0, 0, 0, 10;
    T_C12=T_C11;T_C13=T_C11;T_C14=T_C11;T_C21=T_C11;T_C22=T_C11;T_C23=T_C11;T_C24=T_C11;
    T_C31=T_C11;T_C32=T_C11;T_C33=T_C11;T_C34=T_C11;T_C41=T_C11;T_C42=T_C11;T_C43=T_C11;T_C44=T_C11;


    b = 10;
    
    // fai << imp_rbf_param["fai"].as<Eigen::Matrix<double,4,4>>();
    // Kr = imp_rbf_param["Kr"].as<double>();
    // Kp << imp_rbf_param["Kp"].as<Eigen::Matrix<double,4,4>>();
    // Ki << imp_rbf_param["Ki"].as<Eigen::Matrix<double,4,4>>();
    // Hur << imp_rbf_param["Hur"].as<Eigen::Matrix<double,4,4>>();


    fai << 5, 0, 0, 0,
        0, 5, 0, 0,
        0, 0, 5, 0,
        0, 0, 0, 5;

    Kr = 0.1;

    Kp << 100, 0, 0, 0,
        0, 100, 0, 0,
        0, 0, 100, 0,
        0, 0, 0, 100;
    Ki<< 100, 0, 0, 0,
        0, 100, 0, 0,
        0, 0, 100, 0,
        0, 0, 0, 100;
    Hur << 5, 0, 0, 0,
        0, 5, 0, 0,
        0, 0, 5, 0,
        0, 0, 0, 5;

    double a=2;
    h_M11 << exp(-pow((q - c_M.block(0, 0, 3, 0)).norm(), a )/pow(b,a) ),//norm(), pow()
        exp(-pow((q - c_M.block(0, 1, 3, 0)).norm(), a )/pow(b,a) ),
        exp(-pow((q - c_M.block(0, 2, 3, 0)).norm(), a )/pow(b,a) ),
        exp(-pow((q - c_M.block(0, 3, 3, 0)).norm(), a )/pow(b,a) ),
        exp(-pow((q - c_M.block(0, 4, 3, 0)).norm(), a )/pow(b,a) );

    h_M12=h_M11;h_M13=h_M11;h_M14=h_M11;h_M21=h_M11;h_M22=h_M11;h_M23=h_M11;h_M24=h_M11;
    h_M31=h_M11;h_M32=h_M11;h_M33=h_M11;h_M34=h_M11;h_M41=h_M11;h_M42=h_M11;h_M43=h_M11;h_M44=h_M11;

    h_G1 << exp(-pow((q - c_G.block(0, 0, 3, 0)).norm(), a )/pow(b,a) ),
        exp(-pow((q - c_G.block(0, 1, 3, 0)).norm(), a )/pow(b,a) ),
        exp(-pow((q - c_G.block(0, 2, 3, 0)).norm(), a )/pow(b,a) ),
        exp(-pow((q - c_G.block(0, 3, 3, 0)).norm(), a )/pow(b,a) ),
        exp(-pow((q - c_G.block(0, 4, 3, 0)).norm(), a )/pow(b,a) );
    h_G2=h_G1;h_G3=h_G1;h_G4 = h_G1;


    h_C11 << exp(-pow((z - c_C.block(0, 0, 7, 0)).norm(), a )/pow(b,a) ),
         exp(-pow((z - c_C.block(0, 1, 7, 0)).norm(), a )/pow(b,a) ),
         exp(-pow((z - c_C.block(0, 2, 7, 0)).norm(), a )/pow(b,a) ),
         exp(-pow((z - c_C.block(0, 3, 7, 0)).norm(), a )/pow(b,a) ),
         exp(-pow((z - c_C.block(0, 4, 7, 0)).norm(), a )/pow(b,a) );
    h_C12=h_C11;h_C13=h_C11;h_C14=h_C11;h_C21=h_C11;h_C22=h_C11;h_C23=h_C11;h_C24=h_C11;
    h_C31=h_C11;h_C32=h_C11;h_C33=h_C11;h_C34=h_C11;h_C41=h_C11;h_C42=h_C11;h_C43=h_C11;h_C44=h_C11;
   
    e = qd - q;

    d_e = d_qd - d_q;

    r = d_e + fai * e;

    d_qr = d_qd + fai * e;
    dd_qr = dd_qd + fai * d_e;

    W_M11 << T_M11(0, 0) * h_M11(0, 0) * dd_qr(0, 0) * r(0, 0),
        T_M11(1, 1)* h_M11(1, 0)* dd_qr(0, 0)* r(0, 0),
        T_M11(2, 2)* h_M11(2, 0)* dd_qr(0, 0)* r(0, 0),
        T_M11(3, 3)* h_M11(3, 0)* dd_qr(0, 0)* r(0, 0),
        T_M11(4, 4)* h_M11(4, 0)* dd_qr(0, 0)* r(0, 0);
    W_M12 << T_M12(0, 0) * h_M12(0, 0) * dd_qr(1, 0) * r(0, 0),
        T_M12(1, 1)* h_M12(1, 0)* dd_qr(1, 0)* r(0, 0),
        T_M12(2, 2)* h_M12(2, 0)* dd_qr(1, 0)* r(0, 0),
        T_M12(3, 3)* h_M12(3, 0)* dd_qr(1, 0)* r(0, 0),
        T_M12(4, 4)* h_M12(4, 0)* dd_qr(1, 0)* r(0, 0);
    W_M13 << T_M13(0, 0) * h_M13(0, 0) * dd_qr(2, 0) * r(0, 0),
        T_M13(1, 1)* h_M13(1, 0)* dd_qr(2, 0)* r(0, 0),
        T_M13(2, 2)* h_M13(2, 0)* dd_qr(2, 0)* r(0, 0),
        T_M13(3, 3)* h_M13(3, 0)* dd_qr(2, 0)* r(0, 0),
        T_M13(4, 4)* h_M13(4, 0)* dd_qr(2, 0)* r(0, 0);
    W_M14 << T_M14(0, 0) * h_M14(0, 0) * dd_qr(3, 0) * r(0, 0),
        T_M14(1, 1)* h_M14(1, 0)* dd_qr(3, 0)* r(0, 0),
        T_M14(2, 2)* h_M14(2, 0)* dd_qr(3, 0)* r(0, 0),
        T_M14(3, 3)* h_M14(3, 0)* dd_qr(3, 0)* r(0, 0),
        T_M14(4, 4)* h_M14(4, 0)* dd_qr(3, 0)* r(0, 0);
    W_M21 << T_M21(0, 0) * h_M21(0, 0) * dd_qr(0, 0) * r(1, 0),
        T_M21(1, 1)* h_M21(1, 0)* dd_qr(0, 0)* r(1, 0),
        T_M21(2, 2)* h_M21(2, 0)* dd_qr(0, 0)* r(1, 0),
        T_M21(3, 3)* h_M21(3, 0)* dd_qr(0, 0)* r(1, 0),
        T_M21(4, 4)* h_M21(4, 0)* dd_qr(0, 0)* r(1, 0);
    W_M22 << T_M22(0, 0) * h_M22(0, 0) * dd_qr(1, 0) * r(1, 0),
        T_M22(1, 1)* h_M22(1, 0)* dd_qr(1, 0)* r(1, 0),
        T_M22(2, 2)* h_M22(2, 0)* dd_qr(1, 0)* r(1, 0),
        T_M22(3, 3)* h_M22(3, 0)* dd_qr(1, 0)* r(1, 0),
        T_M22(4, 4)* h_M22(4, 0)* dd_qr(1, 0)* r(1, 0);
    W_M23 << T_M23(0, 0) * h_M23(0, 0) * dd_qr(2, 0) * r(1, 0),
        T_M23(1, 1)* h_M23(1, 0)* dd_qr(2, 0)* r(1, 0),
        T_M23(2, 2)* h_M23(2, 0)* dd_qr(2, 0)* r(1, 0),
        T_M23(3, 3)* h_M23(3, 0)* dd_qr(2, 0)* r(1, 0),
        T_M23(4, 4)* h_M23(4, 0)* dd_qr(2, 0)* r(1, 0);
    W_M24 << T_M24(0, 0) * h_M24(0, 0) * dd_qr(3, 0) * r(1, 0),
        T_M24(1, 1)* h_M24(1, 0)* dd_qr(3, 0)* r(1, 0),
        T_M24(2, 2)* h_M24(2, 0)* dd_qr(3, 0)* r(1, 0),
        T_M24(3, 3)* h_M24(3, 0)* dd_qr(3, 0)* r(1, 0),
        T_M24(4, 4)* h_M24(4, 0)* dd_qr(3, 0)* r(1, 0);
    W_M31 << T_M31(0, 0) * h_M31(0, 0) * dd_qr(0, 0) * r(2, 0),
        T_M31(1, 1)* h_M31(1, 0)* dd_qr(0, 0)* r(2, 0),
        T_M31(2, 2)* h_M31(2, 0)* dd_qr(0, 0)* r(2, 0),
        T_M31(3, 3)* h_M31(3, 0)* dd_qr(0, 0)* r(2, 0),
        T_M31(4, 4)* h_M31(4, 0)* dd_qr(0, 0)* r(2, 0);
    W_M32 << T_M32(0, 0) * h_M32(0, 0) * dd_qr(1, 0) * r(2, 0),
        T_M32(1, 1)* h_M32(1, 0)* dd_qr(1, 0)* r(2, 0),
        T_M32(2, 2)* h_M32(2, 0)* dd_qr(1, 0)* r(2, 0),
        T_M32(3, 3)* h_M32(3, 0)* dd_qr(1, 0)* r(2, 0),
        T_M32(4, 4)* h_M32(4, 0)* dd_qr(1, 0)* r(2, 0);
    W_M33 << T_M33(0, 0) * h_M33(0, 0) * dd_qr(2, 0) * r(2, 0),
        T_M33(1, 1)* h_M33(1, 0)* dd_qr(2, 0)* r(2, 0),
        T_M33(2, 2)* h_M33(2, 0)* dd_qr(2, 0)* r(2, 0),
        T_M33(3, 3)* h_M33(3, 0)* dd_qr(2, 0)* r(2, 0),
        T_M33(4, 4)* h_M33(4, 0)* dd_qr(2, 0)* r(2, 0);
    W_M34 << T_M34(0, 0) * h_M34(0, 0) * dd_qr(3, 0) * r(2, 0),
        T_M34(1, 1)* h_M34(1, 0)* dd_qr(3, 0)* r(2, 0),
        T_M34(2, 2)* h_M34(2, 0)* dd_qr(3, 0)* r(2, 0),
        T_M34(3, 3)* h_M34(3, 0)* dd_qr(3, 0)* r(2, 0),
        T_M34(4, 4)* h_M34(4, 0)* dd_qr(3, 0)* r(2, 0);
    W_M41 << T_M41(0, 0) * h_M41(0, 0) * dd_qr(0, 0) * r(3, 0),
        T_M41(1, 1)* h_M41(1, 0)* dd_qr(0, 0)* r(3, 0),
        T_M41(2, 2)* h_M41(2, 0)* dd_qr(0, 0)* r(3, 0),
        T_M41(3, 3)* h_M41(3, 0)* dd_qr(0, 0)* r(3, 0),
        T_M41(4, 4)* h_M41(4, 0)* dd_qr(0, 0)* r(3, 0);
    W_M42 << T_M42(0, 0) * h_M42(0, 0) * dd_qr(1, 0) * r(3, 0),
        T_M42(1, 1)* h_M42(1, 0)* dd_qr(1, 0)* r(3, 0),
        T_M42(2, 2)* h_M42(2, 0)* dd_qr(1, 0)* r(3, 0),
        T_M42(3, 3)* h_M42(3, 0)* dd_qr(1, 0)* r(3, 0),
        T_M42(4, 4)* h_M42(4, 0)* dd_qr(1, 0)* r(3, 0);
    W_M43 << T_M43(0, 0) * h_M43(0, 0) * dd_qr(2, 0) * r(3, 0),
        T_M43(1, 1)* h_M43(1, 0)* dd_qr(2, 0)* r(3, 0),
        T_M43(2, 2)* h_M43(2, 0)* dd_qr(2, 0)* r(3, 0),
        T_M43(3, 3)* h_M43(3, 0)* dd_qr(2, 0)* r(3, 0),
        T_M43(4, 4)* h_M43(4, 0)* dd_qr(2, 0)* r(3, 0);
    W_M44 << T_M44(0, 0) * h_M44(0, 0) * dd_qr(3, 0) * r(3, 0),
        T_M44(1, 1)* h_M44(1, 0)* dd_qr(3, 0)* r(3, 0),
        T_M44(2, 2)* h_M44(2, 0)* dd_qr(3, 0)* r(3, 0),
        T_M44(3, 3)* h_M44(3, 0)* dd_qr(3, 0)* r(3, 0),
        T_M44(4, 4)* h_M44(4, 0)* dd_qr(3, 0)* r(3, 0);
    W_G3 << T_G3(0, 0) * h_G3(0, 0) * r(0, 0),
        T_G3(1, 1)* h_G3(1, 0)* r(0, 0),
        T_G3(2, 2)* h_G3(2, 0)* r(0, 0),
        T_G3(3, 3)* h_G3(3, 0)* r(0, 0),
        T_G3(4, 4)* h_G3(4, 0)* r(0, 0);
    W_G2 << T_G2(0, 0) * h_G2(0, 0) * r(1, 0),
        T_G2(1, 1)* h_G2(1, 0)* r(1, 0),
        T_G2(2, 2)* h_G2(2, 0)* r(1, 0),
        T_G2(3, 3)* h_G2(3, 0)* r(1, 0),
        T_G2(4, 4)* h_G2(4, 0)* r(1, 0);
    W_G3 << T_G3(0, 0) * h_G3(0, 0) * r(2, 0),
        T_G3(1, 1)* h_G3(1, 0)* r(2, 0),
        T_G3(2, 2)* h_G3(2, 0)* r(2, 0),
        T_G3(3, 3)* h_G3(3, 0)* r(2, 0),
        T_G3(4, 4)* h_G3(4, 0)* r(2, 0);
    W_G4 << T_G4(0, 0) * h_G4(0, 0) * r(3, 0),
        T_G4(1, 1)* h_G4(1, 0)* r(3, 0),
        T_G4(2, 2)* h_G4(2, 0)* r(3, 0),
        T_G4(3, 3)* h_G4(3, 0)* r(3, 0),
        T_G4(4, 4)* h_G4(4, 0)* r(3, 0);
    W_C11 << T_C11(0, 0) * h_C11(0, 0) * d_qr(0, 0) * r(0, 0),
        T_C11(1, 1)* h_C11(1, 0)* d_qr(0, 0)* r(0, 0),
        T_C11(2, 2)* h_C11(2, 0)* d_qr(0, 0)* r(0, 0),
        T_C11(3, 3)* h_C11(3, 0)* d_qr(0, 0)* r(0, 0),
        T_C11(4, 4)* h_C11(4, 0)* d_qr(0, 0)* r(0, 0);
    W_C12 << T_C12(0, 0) * h_C12(0, 0) * d_qr(1, 0) * r(0, 0),
        T_C12(1, 1)* h_C12(1, 0)* d_qr(1, 0)* r(0, 0),
        T_C12(2, 2)* h_C12(2, 0)* d_qr(1, 0)* r(0, 0),
        T_C12(3, 3)* h_C12(3, 0)* d_qr(1, 0)* r(0, 0),
        T_C12(4, 4)* h_C12(4, 0)* d_qr(1, 0)* r(0, 0);
    W_C13 << T_C13(0, 0) * h_C13(0, 0) * d_qr(2, 0) * r(0, 0),
        T_C13(1, 1)* h_C13(1, 0)* d_qr(2, 0)* r(0, 0),
        T_C13(2, 2)* h_C13(2, 0)* d_qr(2, 0)* r(0, 0),
        T_C13(3, 3)* h_C13(3, 0)* d_qr(2, 0)* r(0, 0),
        T_C13(4, 4)* h_C13(4, 0)* d_qr(2, 0)* r(0, 0);
    W_C14 << T_C14(0, 0) * h_C14(0, 0) * d_qr(3, 0) * r(0, 0),
        T_C14(1, 1)* h_C14(1, 0)* d_qr(3, 0)* r(0, 0),
        T_C14(2, 2)* h_C14(2, 0)* d_qr(3, 0)* r(0, 0),
        T_C14(3, 3)* h_C14(3, 0)* d_qr(3, 0)* r(0, 0),
        T_C14(4, 4)* h_C14(4, 0)* d_qr(3, 0)* r(0, 0);
    W_C21 << T_C21(0, 0) * h_C21(0, 0) * d_qr(0, 0) * r(1, 0),
        T_C21(1, 1)* h_C21(1, 0)* d_qr(0, 0)* r(1, 0),
        T_C21(2, 2)* h_C21(2, 0)* d_qr(0, 0)* r(1, 0),
        T_C21(3, 3)* h_C21(3, 0)* d_qr(0, 0)* r(1, 0),
        T_C21(4, 4)* h_C21(4, 0)* d_qr(0, 0)* r(1, 0);
    W_C22 << T_C22(0, 0) * h_C22(0, 0) * d_qr(1, 0) * r(1, 0),
        T_C22(1, 1)* h_C22(1, 0)* d_qr(1, 0)* r(1, 0),
        T_C22(2, 2)* h_C22(2, 0)* d_qr(1, 0)* r(1, 0),
        T_C22(3, 3)* h_C22(3, 0)* d_qr(1, 0)* r(1, 0),
        T_C22(4, 4)* h_C22(4, 0)* d_qr(1, 0)* r(1, 0);
    W_C23 << T_C23(0, 0) * h_C23(0, 0) * d_qr(2, 0) * r(1, 0),
        T_C23(1, 1)* h_C23(1, 0)* d_qr(2, 0)* r(1, 0),
        T_C23(2, 2)* h_C23(2, 0)* d_qr(2, 0)* r(1, 0),
        T_C23(3, 3)* h_C23(3, 0)* d_qr(2, 0)* r(1, 0),
        T_C23(4, 4)* h_C23(4, 0)* d_qr(2, 0)* r(1, 0);
    W_C24 << T_C24(0, 0) * h_C24(0, 0) * d_qr(3, 0) * r(1, 0),
        T_C24(1, 1)* h_C24(1, 0)* d_qr(3, 0)* r(1, 0),
        T_C24(2, 2)* h_C24(2, 0)* d_qr(3, 0)* r(1, 0),
        T_C24(3, 3)* h_C24(3, 0)* d_qr(3, 0)* r(1, 0),
        T_C24(4, 4)* h_C24(4, 0)* d_qr(3, 0)* r(1, 0);
    W_C31 << T_C31(0, 0) * h_C31(0, 0) * d_qr(0, 0) * r(2, 0),
        T_C31(1, 1)* h_C31(1, 0)* d_qr(0, 0)* r(2, 0),
        T_C31(2, 2)* h_C31(2, 0)* d_qr(0, 0)* r(2, 0),
        T_C31(3, 3)* h_C31(3, 0)* d_qr(0, 0)* r(2, 0),
        T_C31(4, 4)* h_C31(4, 0)* d_qr(0, 0)* r(2, 0);
    W_C32 << T_C32(0, 0) * h_C32(0, 0) * d_qr(1, 0) * r(2, 0),
        T_C32(1, 1)* h_C32(1, 0)* d_qr(1, 0)* r(2, 0),
        T_C32(2, 2)* h_C32(2, 0)* d_qr(1, 0)* r(2, 0),
        T_C32(3, 3)* h_C32(3, 0)* d_qr(1, 0)* r(2, 0),
        T_C32(4, 4)* h_C32(4, 0)* d_qr(1, 0)* r(2, 0);
    W_C33 << T_C33(0, 0) * h_C33(0, 0) * d_qr(2, 0) * r(2, 0),
        T_C33(1, 1)* h_C33(1, 0)* d_qr(2, 0)* r(2, 0),
        T_C33(2, 2)* h_C33(2, 0)* d_qr(2, 0)* r(2, 0),
        T_C33(3, 3)* h_C33(3, 0)* d_qr(2, 0)* r(2, 0),
        T_C33(4, 4)* h_C33(4, 0)* d_qr(2, 0)* r(2, 0);
    W_C34 << T_C34(0, 0) * h_C34(0, 0) * d_qr(3, 0) * r(2, 0),
        T_C34(1, 1)* h_C34(1, 0)* d_qr(3, 0)* r(2, 0),
        T_C34(2, 2)* h_C34(2, 0)* d_qr(3, 0)* r(2, 0),
        T_C34(3, 3)* h_C34(3, 0)* d_qr(3, 0)* r(2, 0),
        T_C34(4, 4)* h_C34(4, 0)* d_qr(3, 0)* r(2, 0);
    W_C41 << T_C41(0, 0) * h_C41(0, 0) * d_qr(0, 0) * r(3, 0),
        T_C41(1, 1)* h_C41(1, 0)* d_qr(0, 0)* r(3, 0),
        T_C41(2, 2)* h_C41(2, 0)* d_qr(0, 0)* r(3, 0),
        T_C41(3, 3)* h_C41(3, 0)* d_qr(0, 0)* r(3, 0),
        T_C41(4, 4)* h_C41(4, 0)* d_qr(0, 0)* r(3, 0);
    W_C42 << T_C42(0, 0) * h_C42(0, 0) * d_qr(1, 0) * r(3, 0),
        T_C42(1, 1)* h_C42(1, 0)* d_qr(1, 0)* r(3, 0),
        T_C42(2, 2)* h_C42(2, 0)* d_qr(1, 0)* r(3, 0),
        T_C42(3, 3)* h_C42(3, 0)* d_qr(1, 0)* r(3, 0),
        T_C42(4, 4)* h_C42(4, 0)* d_qr(1, 0)* r(3, 0);
    W_C43 << T_C43(0, 0) * h_C43(0, 0) * d_qr(2, 0) * r(3, 0),
        T_C43(1, 1)* h_C43(1, 0)* d_qr(2, 0)* r(3, 0),
        T_C43(2, 2)* h_C43(2, 0)* d_qr(2, 0)* r(3, 0),
        T_C43(3, 3)* h_C43(3, 0)* d_qr(2, 0)* r(3, 0),
        T_C43(4, 4)* h_C43(4, 0)* d_qr(2, 0)* r(3, 0);
    W_C44 << T_C44(0, 0) * h_C44(0, 0) * d_qr(3, 0) * r(3, 0),
        T_C44(1, 1)* h_C44(1, 0)* d_qr(3, 0)* r(3, 0),
        T_C44(2, 2)* h_C44(2, 0)* d_qr(3, 0)* r(3, 0),
        T_C44(3, 3)* h_C44(3, 0)* d_qr(3, 0)* r(3, 0),
        T_C44(4, 4)* h_C44(4, 0)* d_qr(3, 0)* r(3, 0);

    MSNN << W_M11.transpose() * h_M11, W_M12.transpose()* h_M12, W_M13.transpose()* h_M13, W_M14.transpose()* h_M14,
        W_M21.transpose()* h_M21, W_M22.transpose()* h_M22, W_M23.transpose()* h_M23, W_M24.transpose()* h_M24,
        W_M31.transpose()* h_M31, W_M32.transpose()* h_M32, W_M33.transpose()* h_M33, W_M34.transpose()* h_M34,
        W_M41.transpose()* h_M41, W_M42.transpose()* h_M42, W_M13.transpose()* h_M43, W_M14.transpose()* h_M44;
    GSNN << W_G1.transpose() * h_G1,
        W_G2.transpose()* h_G2,
        W_G3.transpose()* h_G3,
        W_G4.transpose()* h_G4;
    CDNN << W_C11.transpose() * h_C11, W_C12.transpose()* h_C12, W_C13.transpose()* h_C13, W_C14.transpose()* h_C14,
        W_C21.transpose()* h_C21, W_C22.transpose()* h_C22, W_C23.transpose()* h_C23, W_C24.transpose()* h_C24,
        W_C31.transpose()* h_C31, W_C32.transpose()* h_C32, W_C33.transpose()* h_C33, W_C34.transpose()* h_C34,
        W_C41.transpose()* h_C41, W_C42.transpose()* h_C42, W_C13.transpose()* h_C43, W_C14.transpose()* h_C44;
    }
}



#endif
