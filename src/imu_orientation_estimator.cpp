#include <imu_orientation_estimator.h>

void imu_orientation_estimator::TF_(const Eigen::Matrix3d& G_R_I){
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id="base_link";
    tfs.header.stamp=ros::Time::now();
    tfs.child_frame_id="imu";

    Eigen::Vector3d eulerAngle_sum=G_R_I.eulerAngles(2,1,0);
    tf2::Quaternion qtn;
    qtn.setRPY(eulerAngle_sum[2],eulerAngle_sum[1],eulerAngle_sum[0]);//roll pitch yaw
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();
    broadcaster.sendTransform(tfs);
}

//构造
imu_orientation_estimator::imu_orientation_estimator(const std::string& topic_imu,  double t, const Eigen::Matrix3d& Acc_noise, const Eigen::Matrix<double,6,6>& Begin_cov){
    sub_imu = nh_p.subscribe<sensor_msgs::Imu>(topic_imu, 200, &imu_orientation_estimator::IMU_Handle, this, ros::TransportHints().tcpNoDelay());
    this->delta_t = t;
    this->acc_noise = Acc_noise;
    this->begin_cov = Begin_cov;
   
}

//IMU_Handle()
void imu_orientation_estimator::IMU_Handle(const sensor_msgs::Imu::ConstPtr& msg_imu){

    //位姿初始化
    acc<<msg_imu->linear_acceleration.x, msg_imu->linear_acceleration.y, msg_imu->linear_acceleration.z;
    gyro<<msg_imu->angular_velocity.x, msg_imu->angular_velocity.y, msg_imu->angular_velocity.z;
    //如果没有初始化就进入初始化,初始化失败，打印相关信息，退出handle；初始化位姿G_R_I，
    if (config_initializer.initialize_flag == false){
        if(Initialize(acc)==false)  return;
    }

    //Propagate   G_R_I begin_bg begin_cov                      输出end_cov     end_G_R_I     end_bg
    PropagateMeanAndCov(G_R_I, begin_bg, begin_cov, gyro, delta_t);

    Updata(end_G_R_I, end_bg, end_cov, acc, acc_noise);

    // //TF
    TF_(G_R_I);

    return ;
}


//位姿初始化
bool imu_orientation_estimator::Initialize(const Eigen::Vector3d& acc){
    acc_buffer.push_back(acc);
    if(acc_buffer.size()<=config_initializer.acc_buffer_size)   return false;
    acc_buffer.pop_front();

    //计算mean acc
    Eigen::Vector3d mean_acc;
    for (const Eigen::Vector3d& one_acc : acc_buffer) {
        mean_acc += one_acc;
    }
    mean_acc = mean_acc / static_cast<double>(acc_buffer.size());

    //计算 acc标准差
    Eigen::Vector3d std_acc;
    for (const Eigen::Vector3d& one_acc : acc_buffer){
        std_acc += (one_acc - mean_acc).cwiseAbs2();//cwiseAbs2 求元素的平方
    }
    std_acc = (std_acc / static_cast<double>(acc_buffer.size())).cwiseSqrt();//求元素的标准差
    if (std_acc.norm() > config_initializer.max_acc_std){//std_acc_x^2+std_acc_y^2+std_acc_z^2=std_acc.norm(),求解的宗方差
        std::cout << "[Initialize]: Initializaion failed. Too big acc std: " << std::fixed << std_acc.transpose();//std::fixed目的打印所有的有效数字
        return false;
    }

    //Get initial orientation
    const Eigen::Vector3d z_axis = mean_acc.normalized();
    const Eigen::Vector3d x_axis = 
        (Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX()).normalized();
    const Eigen::Vector3d y_axis = (z_axis.cross(x_axis)).normalized();
    
    Eigen::Matrix3d I_R_G;
    I_R_G.block<3, 1>(0, 0) = x_axis;
    I_R_G.block<3, 1>(0, 1) = y_axis;
    I_R_G.block<3, 1>(0, 2) = z_axis;

    G_R_I = I_R_G.transpose();//得到姿态的初始估计

    config_initializer.initialize_flag=true;

    return true;
    
}

//状态传播方程
void imu_orientation_estimator::PropagateMeanAndCov(const Eigen::Matrix3d& begin_G_R_I,
                                                    const Eigen::Vector3d& begin_bg,
                                                    const Eigen::Matrix<double, 6, 6>& begin_cov,
                                                    const Eigen::Vector3d& gyro,
                                                    const double delta_t)
{
    //Mean propagate
    const Eigen::Vector3d unbiased_gyro = gyro - begin_bg;
    const Eigen::Vector3d angle_vec = unbiased_gyro * delta_t;
    Eigen::Matrix3d delta_rot;

    if(angle_vec.norm() < 1e-12){
        delta_rot=Eigen::Quaterniond(Eigen::Matrix3d::Identity() + SkewMat(angle_vec)).normalized().toRotationMatrix();
    }else{
        const double angle = angle_vec.norm();
        const Eigen::Vector3d axis = angle_vec/angle;
        delta_rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    }
    end_G_R_I = begin_G_R_I * delta_rot;;
    end_bg = begin_bg;

    //计算雅各比，更新协方差
    //Fx 
    Eigen::Matrix<double, 6, 6> Fx;
    Fx.topLeftCorner<3, 3>() = delta_rot.transpose();
    Fx.topRightCorner<3,3>() = -Eigen::Matrix3d::Identity() * delta_t;
    Fx.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Zero();
    Fx.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity();

    //Q  传感器测量的协方差矩阵//
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * gyro_noise * delta_t * delta_t;//状态转移方程的协方差矩阵  陀螺仪的噪声（对w影响）
    Q.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * gyro_bias_noise* delta_t;

    end_cov = Fx * begin_cov * Fx.transpose() + Q;
    return ;
}

//Update更新部分
void imu_orientation_estimator::Updata(const Eigen::Matrix3d prior_G_R_I,
            const Eigen::Vector3d &prior_bg,
            const Eigen::Matrix<double, 6, 6>& prior_cov,
            const Eigen::Vector3d &acc,
            const Eigen::Matrix3d &acc_noise)
{
    //Residual
    Eigen::Vector3d gravity_vec(0., 0., 1.);
    Eigen::Vector3d residual = acc.normalized() - prior_G_R_I.transpose() * gravity_vec;
    // Jacobian.
    Eigen::Matrix<double, 3, 6> H;
    H.setZero();
    H.block<3, 3>(0, 0) = SkewMat(prior_G_R_I.transpose() * gravity_vec);
    // Kalman gain.
    Eigen::Matrix<double, 6, 3> K = prior_cov * H.transpose() * (H * prior_cov * H.transpose() + acc_noise).inverse();
    // Delta x.
    Eigen::Matrix<double, 6, 1> delta_x = K * residual;
    // Update state.
    Eigen::Matrix3d delta_Rot;
    if (delta_x.topRows<3>().norm() < 1e-12) {
        delta_Rot = 
            Eigen::Quaterniond(
                Eigen::Matrix3d::Identity() + SkewMat(delta_x.topRows<3>())).normalized().toRotationMatrix();
    } else {
        const double angle = delta_x.topRows<3>().norm();
        const Eigen::Vector3d axis = delta_x.topRows<3>() / angle;
        delta_Rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    }
    G_R_I = prior_G_R_I * delta_Rot;
    begin_bg = prior_bg + delta_x.bottomRows<3>();

    // Update covariance.
    const Eigen::Matrix<double, 6, 6> I_mins_KH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
    begin_cov = I_mins_KH * prior_cov * I_mins_KH.transpose() + K * acc_noise * K.transpose();
}

//utility
Eigen::Matrix3d imu_orientation_estimator::SkewMat(const Eigen::Vector3d& v){
    Eigen::Matrix3d w;//构建反对称矩阵
    w <<  0.,   -v(2),  v(1),
          v(2),  0.,   -v(0),
         -v(1),  v(0),  0.;
    return w;
}
