#pragma once

#include <iostream>
#include <ros/ros.h>
#include <deque>
#include <Eigen/Core>
#include  <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>


class imu_orientation_estimator{
    public:
    imu_orientation_estimator(const std::string& topic_imu,  double  t, const Eigen::Matrix3d& Acc_noise, const Eigen::Matrix<double,6,6>& Begin_cov);

    //utility
    Eigen::Matrix3d SkewMat(const Eigen::Vector3d& v);

    void IMU_Handle(const sensor_msgs::Imu::ConstPtr& msg_imu);

    bool Initialize(const Eigen::Vector3d& acc);
   
//
    void PropagateMeanAndCov(const Eigen::Matrix3d& begin_G_R_I, 
                             const Eigen::Vector3d& begin_bg, 
                             const Eigen::Matrix<double, 6, 6>& begin_cov,
                             const Eigen::Vector3d& gyro, 
                             const double delta_t);
 
//根据之前的先验估计，得到新的状态量的后验估计、新的误差协方差矩阵
    void Updata(const Eigen::Matrix3d prior_G_R_I,
                                const Eigen::Vector3d& prior_bg,
                                const Eigen::Matrix<double, 6, 6>& prior_cov,
                                const Eigen::Vector3d& acc,
                                const Eigen::Matrix3d& acc_noise);

    //TF
    void TF_(const Eigen::Matrix3d& G_R_I);

    private:
    tf2_ros::TransformBroadcaster broadcaster;
    ros::NodeHandle nh_p;
    ros::Subscriber sub_imu;
    ros::Publisher pub_odom;

    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;

    std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> acc_buffer;
    std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> gyro_buffer;


    //----------------------初始化部分----------------
    struct Config_Initialize {
        int acc_buffer_size = 10;
        double max_acc_std = 0.5;
        bool initialize_flag = false;
    }config_initializer;

    Eigen::Matrix3d G_R_I;//姿态量的初始估计，给定初始的位姿估计

    //----------------------------状态传递方程部分        输入量是begin_G_R_I、begin_bg、begin_cov、给定gyro（角速度）、delta_t-----------------
         //Eigen::Matrix3d G_R_I;       //给定初始的状态量 
        Eigen::Vector3d begin_bg = Eigen::Vector3d::Zero();//给定初始的bg(手动给定)
        double delta_t;
        Eigen::Matrix<double, 6, 6> begin_cov;//给定初始的误差协方差矩阵

        const double gyro_noise = 1e-6;//陀螺仪的噪声
        const double gyro_bias_noise = 1e-8;//陀螺仪bias的噪声

        //先验估计值
        Eigen::Matrix3d end_G_R_I;
        Eigen::Vector3d end_bg;
        Eigen::Matrix<double,6,6> end_cov;//先验估计误差协方差

    //--------------------------------更新部分 对G_R_I、begin_bg、begin_cov进行更新----------------------
        Eigen::Matrix3d acc_noise;
};