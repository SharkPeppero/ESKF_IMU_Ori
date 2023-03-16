#include <iostream>
#include <ros/ros.h>
#include <imu_orientation_estimator.h>

int main(int argc, char*argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"imu_orientation_estimator");
    ros::NodeHandle nh;
    //读入yaml文件
    std::string topic_imu;         nh.getParam("topic_imu", topic_imu);//topic_imu

    double delta_t;                     nh.getParam("delta_t",delta_t);//delta_t

    //acc_noise
    std::vector<double> v_acc_noise;        nh.getParam("acc_noise", v_acc_noise);
    Eigen::Matrix3d acc_noise;          acc_noise = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(v_acc_noise.data()); 
    ROS_WARN_STREAM("acc_noise:\n"<<acc_noise.matrix());

    //begin_cov
    std::vector<double> v_begin_cov;        nh.getParam("begin_cov", v_begin_cov);
    Eigen::Matrix<double, 6, 6> begin_cov;               begin_cov = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(v_begin_cov.data());

    ROS_WARN_STREAM("begin_cov:\n"<<begin_cov.matrix());


    imu_orientation_estimator p(topic_imu, delta_t, acc_noise, begin_cov);

    ros::spin();

    // ros::MultiThreadedSpinner spinner(3);
    // spinner.spin();


    return 0;
}
