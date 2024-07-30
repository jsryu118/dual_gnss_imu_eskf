#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <Eigen/Core>
#include <deque>
#include <fstream>
#include <chrono>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include "imu_gnss_fusion/kf.h"
#include "imu_gnss_fusion/utils.h"

namespace ryu {

class FusionNode {
   public:
    FusionNode(ros::NodeHandle &nh) {
        //////////////////gnss///////////////////////
        // default : UNIST
        double local_long, local_lat, local_alt;
        nh.param("local_long", local_long, 35.571127);
        nh.param("local_lat", local_lat, 129.1874088);
        nh.param("local_alt", local_alt, 76.716);
        origin_enu_ = Eigen::Vector3d(local_long, local_lat, local_alt);

        std::string topic_gnss_front, topic_gnss_behind;
        double gnss_f_x_imuFrame, gnss_f_y_imuFrame, gnss_f_z_imuFrame;
        double gnss_b_x_imuFrame, gnss_b_y_imuFrame, gnss_b_z_imuFrame;
        nh.param<std::string>("topic_gnss_front", topic_gnss_front, "/gnss_2/llh_position");
        nh.param<std::string>("topic_gnss_behind", topic_gnss_behind, "/gnss_1/llh_position");
        nh.param("gnss_f_x_imuFrame", gnss_f_x_imuFrame, 0.);
        nh.param("gnss_f_y_imuFrame", gnss_f_y_imuFrame, 0.);
        nh.param("gnss_f_z_imuFrame", gnss_f_z_imuFrame, 0.);
        nh.param("gnss_b_x_imuFrame", gnss_b_x_imuFrame, 0.);
        nh.param("gnss_b_y_imuFrame", gnss_b_y_imuFrame, 0.);
        nh.param("gnss_b_z_imuFrame", gnss_b_z_imuFrame, 0.);
        I_p_gnss_f_ = Eigen::Vector3d(gnss_f_x_imuFrame, gnss_f_y_imuFrame, gnss_f_z_imuFrame);
        I_p_gnss_b_ = Eigen::Vector3d(gnss_b_x_imuFrame, gnss_b_y_imuFrame, gnss_b_z_imuFrame);


        //////////////////IMU///////////////////////        
        std::string topic_imu;
        nh.param<std::string>("topic_imu", topic_imu, "/imu/data");
        double acc_n, gyr_n, acc_w, gyr_w;
        nh.param("acc_noise", acc_n, 1e-2);
        nh.param("gyr_noise", gyr_n, 1e-4);
        nh.param("acc_bias_noise", acc_w, 1e-6);
        nh.param("gyr_bias_noise", gyr_w, 1e-8);

        ////////////////// KF ///////////////////////  
        kf_ptr_ = std::make_unique<KF>(acc_n, gyr_n, acc_w, gyr_w);
        delay_kf_ptr_ = std::make_unique<KF>(acc_n, gyr_n, acc_w, gyr_w);

        ///////////////INITIALIZATION////////////////
        nh.param("GnssBufSize", kGnssBufSize, 5);
        nh.param("DelayImuBufSize", kDelayImuBufSize, 20);
        nh.param("InitImuBufSize", kInitImuBufSize, 100);
        nh.param("acc_std_threshold", acc_std_threshold, 3.);
        nh.param("heading_std_threshold", heading_std_threshold, 3.);
        nh.param("delta_gnss_threshold", delta_gnss_threshold, 0.05);
        nh.param("time_sync_threshold", time_sync_threshold, 0.05);

        //////////////////ODOMETRY///////////////////////  
        std::string topic_odometry;
        nh.param<std::string>("topic_odometry", topic_odometry, "/eskf/odom");
        double baes_link_x_imuFrame, baes_link_y_imuFrame, baes_link_z_imuFrame,
                baes_link_qx_imuFrame, baes_link_qy_imuFrame, baes_link_qz_imuFrame, baes_link_qw_imuFrame;
        nh.param("baes_link_x_imuFrame", baes_link_x_imuFrame, 0.);
        nh.param("baes_link_y_imuFrame", baes_link_y_imuFrame, 0.);
        nh.param("baes_link_z_imuFrame", baes_link_z_imuFrame, 0.);
        nh.param("baes_link_qx_imuFrame", baes_link_qx_imuFrame, 0.);
        nh.param("baes_link_qy_imuFrame", baes_link_qy_imuFrame, 0.);
        nh.param("baes_link_qz_imuFrame", baes_link_qz_imuFrame, 0.);
        nh.param("baes_link_qw_imuFrame", baes_link_qw_imuFrame, 1.);
        Eigen::Quaterniond quaternion(baes_link_qw_imuFrame, baes_link_qx_imuFrame, baes_link_qy_imuFrame, baes_link_qz_imuFrame);
        r_imu_2_baselink = quaternion.toRotationMatrix();
        Eigen::Vector3d translation_vector(baes_link_x_imuFrame, baes_link_y_imuFrame, baes_link_z_imuFrame);
        t_imu_2_baselink = translation_vector;

        ////////////////// TF ///////////////////////  
        nh.param("pub_tf", pub_tf, false);
        nh.param<std::string>("frame_id", frame_id, "map");
        nh.param<std::string>("child_frame_id", child_frame_id, "base_link");

        ////////////////// OTHERS ///////////////////////      
        std::string topic_path;
        nh.param<std::string>("topic_path", topic_path, "/eskf_path");
        nh.param("pub_path", pub_path, false);

        ////////////////// ROS Sub && Pub///////////////////////  
        imu_sub_ = nh.subscribe(topic_imu, 10, &FusionNode::imu_callback, this);
        gnss_f_sub_ = nh.subscribe(topic_gnss_front, 10, &FusionNode::gnss_f_callback, this);
        gnss_b_sub_ = nh.subscribe(topic_gnss_behind, 10, &FusionNode::gnss_b_callback, this);

        path_pub_ = nh.advertise<nav_msgs::Path>(topic_path, 10);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>(topic_odometry, 10);
    }

    ~FusionNode() {
    }

    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
    void gnss_f_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg);
    void initialize_heading();
    void gnss_b_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg);
    void gnss_process(const gnssDataPtr gnss_data_ptr, const Eigen::Vector3d& I_p_gnss_);
    void delay_gnss_process(const gnssDataPtr gnss_data_ptr, const Eigen::Vector3d& I_p_gnss_);
    
    bool init_rot_from_imudata(Eigen::Matrix3d &r_GI);
    void publish_save_state();

   private:
    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_f_sub_;
    ros::Subscriber gnss_b_sub_;
    ros::Publisher path_pub_;
    ros::Publisher odom_pub_;

    tf::TransformBroadcaster tf_broadcaster_;

    nav_msgs::Path nav_path_;

    // rosparam
    std::string frame_id;
    std::string child_frame_id;
    bool pub_tf;
    double acc_std_threshold;
    double heading_std_threshold;
    double delta_gnss_threshold;
    double time_sync_threshold;
    bool pub_path;

    // init
    bool initialized_rot_ = false;
    bool initialized_gnss_ = false;
    bool initialized_heading_ = false;
    int kInitImuBufSize;
    int kDelayImuBufSize;
    int kGnssBufSize; 
    std::deque<ImuDataConstPtr> init_imu_buf_;
    std::deque<ImuDataConstPtr> delay_imu_buf_;
    std::deque<StatePtr> delay_state_buf_;
    std::deque<gnssDataConstPtr> gnss_f_buf_;
    std::deque<gnssDataConstPtr> gnss_b_buf_;
    ImuDataConstPtr last_imu_ptr_;
    Eigen::Vector3d origin_enu_;
    Eigen::Vector3d I_p_gnss_f_;
    Eigen::Vector3d I_p_gnss_b_;
    Eigen::Vector3d initial_heading_enu;

    bool imu_start_flag = false;
    bool init_end_flag = false;
    double init_start;
    double init_end;

    // imu to base_link

    Eigen::Matrix3d r_imu_2_baselink;
    Eigen::Vector3d t_imu_2_baselink;

    // KF
    KFPtr kf_ptr_;
    KFPtr delay_kf_ptr_;
};

void FusionNode::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
    if (!imu_start_flag) {
        init_start = imu_msg->header.stamp.toSec();
        imu_start_flag = true;
    }
    ImuDataPtr imu_data_ptr = std::make_shared<ImuData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
    imu_data_ptr->acc[1] = imu_msg->linear_acceleration.y;
    imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
    imu_data_ptr->gyr[0] = imu_msg->angular_velocity.x;
    imu_data_ptr->gyr[1] = imu_msg->angular_velocity.y;
    imu_data_ptr->gyr[2] = imu_msg->angular_velocity.z;

    // delay_imu_buf_.push_back(imu_data_ptr);
    // if (delay_imu_buf_.size() > kDelayImuBufSize) delay_imu_buf_.pop_front();

    if (!initialized_gnss_) {
        init_imu_buf_.push_back(imu_data_ptr);
        if (init_imu_buf_.size() > kInitImuBufSize) init_imu_buf_.pop_front();
    }

    if (!initialized_heading_) {
        if (gnss_f_buf_.size() > kGnssBufSize-1 && gnss_b_buf_.size() > kGnssBufSize-1 ){
            initialize_heading();
        }
    }

    if (initialized_gnss_ && initialized_heading_){
        if (!init_end_flag) {
            init_end = imu_msg->header.stamp.toSec();
            double initialization_time = init_end - init_start;
            printf("[gnss_imu %s] Initialization takes %f secs!!!!\n", __FUNCTION__, initialization_time);
            init_end_flag = true;
        }

        delay_imu_buf_.push_back(last_imu_ptr_);
        delay_state_buf_.push_back(std::make_shared<State>(*kf_ptr_->state_ptr_));
        if (delay_imu_buf_.size() > kDelayImuBufSize) delay_imu_buf_.pop_front();
        if (delay_state_buf_.size() > kDelayImuBufSize) delay_state_buf_.pop_front();

        kf_ptr_->predict(last_imu_ptr_, imu_data_ptr);

        publish_save_state();


    }

    last_imu_ptr_ = imu_data_ptr;

}

void FusionNode::initialize_heading() {
    Eigen::Vector3d sum_front(0., 0., 0.);
    Eigen::Vector3d sum_behind(0., 0., 0.);

    std::vector<Eigen::Vector3d> gnss_f_enu_vec;
    std::vector<Eigen::Vector3d> gnss_b_enu_vec;

    for (const auto gnss_f : gnss_f_buf_) {
        Eigen::Vector3d gnss_f_enu; // position of gnss from global(map) frame
        ryu::lla2enu(origin_enu_, gnss_f->lla, &gnss_f_enu);
        sum_front += gnss_f_enu;
        gnss_f_enu_vec.push_back(gnss_f_enu);
    }
    for (const auto gnss_b : gnss_b_buf_) {
        Eigen::Vector3d gnss_b_enu; // position of gnss from global(map) frame
        ryu::lla2enu(origin_enu_, gnss_b->lla, &gnss_b_enu);
        sum_behind += gnss_b_enu;
        gnss_b_enu_vec.push_back(gnss_b_enu);
    }

    const Eigen::Vector3d mean_front = sum_front / (double)gnss_f_buf_.size();
    const Eigen::Vector3d mean_behind = sum_behind / (double)gnss_b_buf_.size();

    Eigen::Vector3d sum_err2_f(0., 0., 0.);
    Eigen::Vector3d sum_err2_b(0., 0., 0.);
    for (const auto gnss_f_enu : gnss_f_enu_vec) sum_err2_f += (gnss_f_enu - mean_front).cwiseAbs2();
    for (const auto gnss_b_enu : gnss_b_enu_vec) sum_err2_b += (gnss_b_enu - mean_behind).cwiseAbs2();
    const Eigen::Vector3d std_gnss_f = (sum_err2_f / (double)gnss_f_buf_.size()).cwiseSqrt();
    const Eigen::Vector3d std_gnss_b = (sum_err2_b / (double)gnss_b_buf_.size()).cwiseSqrt();

    bool std_flag_f_ = true;
    bool std_flag_b_ = true;
    if (std_gnss_f.maxCoeff() > heading_std_threshold) {
        printf("[gnss_imu %s] Too big gnss_front_std for heading initialization: (%f, %f, %f)!!!\n", __FUNCTION__, std_gnss_f[0], std_gnss_f[1], std_gnss_f[2]);
        gnss_f_buf_.clear();
        std_flag_f_ = false;
    }
    if (std_gnss_b.maxCoeff() > heading_std_threshold) {
        printf("[gnss_imu %s] Too big gnss_behind_std for heading initialization: (%f, %f, %f)!!!\n", __FUNCTION__, std_gnss_b[0], std_gnss_b[1], std_gnss_b[2]);
        gnss_b_buf_.clear();
        std_flag_b_ = false;
    }
    if (!(std_flag_f_ && std_flag_b_)) return;

    double measured_distance = (mean_front - mean_behind).norm();
    double estimated_distance = (I_p_gnss_f_-I_p_gnss_b_).norm();
    double distance_err = std::abs(measured_distance - estimated_distance);

    if (distance_err > delta_gnss_threshold){
        printf("[gnss_imu %s] two gnss are not located within estimated distance! : %f m !!!\n", __FUNCTION__, distance_err);
        gnss_f_buf_.clear();
        gnss_b_buf_.clear();
        return;
    }

    initial_heading_enu = mean_front - mean_behind;
    initialized_heading_ = true;
}

void FusionNode::gnss_f_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg) {
    // status 2:ground based fix
    // status 0:no fix
    if (gnss_msg->status.status != 2) {
        printf("[gnss_imu %s] ERROR: Bad gnss Message!!!\n", __FUNCTION__);
        return;
    }

    gnssDataPtr gnss_data_ptr = std::make_shared<gnssData>();
    gnss_data_ptr->timestamp = gnss_msg->header.stamp.toSec();
    gnss_data_ptr->lla[0] = gnss_msg->latitude;
    gnss_data_ptr->lla[1] = gnss_msg->longitude;
    gnss_data_ptr->lla[2] = gnss_msg->altitude;
    gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gnss_msg->position_covariance.data());

    if (!initialized_heading_) {
        gnss_f_buf_.push_back(gnss_data_ptr);
        if (gnss_f_buf_.size() > kGnssBufSize) gnss_f_buf_.pop_front();
        return;
    }
    
    // gnss_process(gnss_data_ptr, I_p_gnss_f_);
}

void FusionNode::gnss_b_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg) {
    // status 2:ground based fix
    // status 0:no fix
    if (gnss_msg->status.status != 2) {
        printf("[gnss_imu %s] ERROR: Bad gnss Message!!!\n", __FUNCTION__);
        return;
    }

    gnssDataPtr gnss_data_ptr = std::make_shared<gnssData>();
    gnss_data_ptr->timestamp = gnss_msg->header.stamp.toSec();
    gnss_data_ptr->lla[0] = gnss_msg->latitude;
    gnss_data_ptr->lla[1] = gnss_msg->longitude;
    gnss_data_ptr->lla[2] = gnss_msg->altitude;
    gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gnss_msg->position_covariance.data());

    if (!initialized_heading_) {
        gnss_b_buf_.push_back(gnss_data_ptr);
        if (gnss_b_buf_.size() > kGnssBufSize) gnss_b_buf_.pop_front();
        return;
    }

    if (!initialized_gnss_) gnss_process(gnss_data_ptr, I_p_gnss_b_);
    delay_gnss_process(gnss_data_ptr, I_p_gnss_b_);
}

void FusionNode::gnss_process(const gnssDataPtr gnss_data_ptr, const Eigen::Vector3d& I_p_gnss_) {
    if (!initialized_rot_) {
        if (init_imu_buf_.size() < kInitImuBufSize) {
            printf("[gnss_imu %s] ERROR: Not Enough IMU data for Initialization!!!\n", __FUNCTION__);
            return;
        }

        last_imu_ptr_ = init_imu_buf_.back();
        if (std::abs(gnss_data_ptr->timestamp - last_imu_ptr_->timestamp) > time_sync_threshold) {
            printf("[gnss_imu %s] ERROR: gnss and Imu timestamps are not synchronized!!!\n", __FUNCTION__);
            return;
        }

        kf_ptr_->state_ptr_->timestamp = last_imu_ptr_->timestamp;

        if (!init_rot_from_imudata(kf_ptr_->state_ptr_->r_GI)) return;

        // origin_enu_ = gnss_data_ptr->lla;

        initialized_rot_ = true;

        printf("[gnss_imu %s] System initialized.\n", __FUNCTION__);

        return;
    }
    // convert WGS84 to ENU frame
    Eigen::Vector3d p_G_gnss; // position of gnss from global(map) frame
    ryu::lla2enu(origin_enu_, gnss_data_ptr->lla, &p_G_gnss);

    const Eigen::Vector3d &p_GI = kf_ptr_->state_ptr_->p_GI;
    const Eigen::Matrix3d &r_GI = kf_ptr_->state_ptr_->r_GI;

    // residual
    Eigen::Vector3d residual = (p_G_gnss - r_GI * I_p_gnss_) - p_GI;
    double abs_residual = residual.norm();

    // jacobian
    Eigen::Matrix<double, 3, 15> H;
    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(0, 6) = -r_GI * ryu::skew_matrix(I_p_gnss_);

    // measurement covariance
    const Eigen::Matrix3d &V = gnss_data_ptr->cov;

    kf_ptr_->update_measurement(H, V, residual);

    if (!initialized_gnss_)  initialized_gnss_ = true;
}

void FusionNode::delay_gnss_process(const gnssDataPtr gnss_data_ptr, const Eigen::Vector3d& I_p_gnss_) {
    if (!initialized_rot_) {
        if (init_imu_buf_.size() < kInitImuBufSize) {
            printf("[gnss_imu %s] ERROR: Not Enough IMU data for Initialization!!!\n", __FUNCTION__);
            return;
        }

        last_imu_ptr_ = init_imu_buf_.back();
        if (std::abs(gnss_data_ptr->timestamp - last_imu_ptr_->timestamp) > time_sync_threshold) {
            printf("[gnss_imu %s] ERROR: gnss and Imu timestamps are not synchronized!!!\n", __FUNCTION__);
            return;
        }

        kf_ptr_->state_ptr_->timestamp = last_imu_ptr_->timestamp;

        if (!init_rot_from_imudata(kf_ptr_->state_ptr_->r_GI)) return;

        // origin_enu_ = gnss_data_ptr->lla;

        initialized_rot_ = true;

        printf("[gnss_imu %s] System initialized.\n", __FUNCTION__);

        return;
    }
    // convert WGS84 to ENU frame
    // std::cout << delay_imu_buf_.size()<< std::endl;
    if (delay_imu_buf_.size() >= kDelayImuBufSize){
        // auto start = std::chrono::high_resolution_clock::now();

        Eigen::Vector3d p_G_gnss; // position of gnss from global(map) frame
        ryu::lla2enu(origin_enu_, gnss_data_ptr->lla, &p_G_gnss);

        StatePtr first_state_ptr = delay_state_buf_.front();
        // StatePtr last_state_ptr = delay_state_buf_[10];
        // StatePtr last_state_ptr = delay_state_buf_[10];
        // std::cout << "First state p_GI: " << first_state_ptr->p_GI.transpose() << std::endl;
        // std::cout << "Last state p_GI: " << last_state_ptr->p_GI.transpose() << std::endl;
        // std::cout << "Last state p_GI: " << last_state_ptr->p_GI.transpose() << std::endl;

        const Eigen::Vector3d &p_GI = first_state_ptr->p_GI;
        const Eigen::Matrix3d &r_GI = first_state_ptr->r_GI;

        // residual
        Eigen::Vector3d residual = (p_G_gnss - r_GI * I_p_gnss_) - p_GI;

        double abs_residual = residual.norm();

        // jacobian
        Eigen::Matrix<double, 3, 15> H;
        H.setZero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        H.block<3, 3>(0, 6) = -r_GI * ryu::skew_matrix(I_p_gnss_);

        // measurement covariance
        const Eigen::Matrix3d &V = gnss_data_ptr->cov;
        update_delay_measurement(first_state_ptr, H, V, residual);
        delay_kf_ptr_->state_ptr_ = first_state_ptr;

        for (size_t i = 1; i < delay_imu_buf_.size(); ++i) {
            delay_kf_ptr_->predict(delay_imu_buf_[i-1], delay_imu_buf_[i]);
        }
        // std::cout << delay_kf_ptr_->state_ptr_->p_GI<< std::endl;


        // first_state_ptr가 predict된 값과 현재 state_ptr와 비교하여 최종적으로 업데이트 하라.
        // update_measurement는 position 만 고려하지만 rotation 까지 고려하도록 만들어라.
        Eigen::Vector3d new_residual = delay_kf_ptr_->state_ptr_->p_GI - kf_ptr_->state_ptr_->p_GI;
        const Eigen::Vector3d &new_p_GI = kf_ptr_->state_ptr_->p_GI;
        const Eigen::Matrix3d &new_r_GI = kf_ptr_->state_ptr_->r_GI;
        // jacobian
        Eigen::Matrix<double, 3, 15> new_H;
        new_H.setZero();
        new_H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        new_H.block<3, 3>(0, 6) = -new_r_GI * ryu::skew_matrix(I_p_gnss_);

        // measurement covariance
        const Eigen::Matrix3d &new_V = first_state_ptr->cov.block<3, 3>(0, 0);
        kf_ptr_->update_measurement(new_H, new_V, new_residual);


        // Clear the buffers as we have processed the delayed data
        // delay_state_buf_.clear();
        // delay_imu_buf_.clear();
    //     auto end = std::chrono::high_resolution_clock::now();
    //     std::chrono::duration<double> elapsed = end - start;

    // // 걸린 시간 출력
    //     std::cout << "If statement took " << elapsed.count() << " seconds to execute." << std::endl;
    }

        // if (!initialized_gnss_)  initialized_gnss_ = true;
}

bool FusionNode::init_rot_from_imudata(Eigen::Matrix3d &r_GI) {
    // mean and std of IMU accs
    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (const auto imu_data : init_imu_buf_) {
        sum_acc += imu_data->acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)init_imu_buf_.size();
    // printf("[gnss_imu %s] mean_acc: (%f, %f, %f)!!!\n", __FUNCTION__, mean_acc[0], mean_acc[1], mean_acc[2]);

    // std::cout << (mean_acc.cwiseAbs2()).cwiseSqrt() <<std::endl;
    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (const auto imu_data : init_imu_buf_) sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    const Eigen::Vector3d std_acc = (sum_err2 / (double)init_imu_buf_.size()).cwiseSqrt();

    // acc std limit: 3
    if (std_acc.maxCoeff() > acc_std_threshold) {
        printf("[gnss_imu %s] Too big acc std: (%f, %f, %f)!!!\n", __FUNCTION__, std_acc[0], std_acc[1], std_acc[2]);
        return false;
    }

    // Compute rotation.
    // ref: https://github.com/rpng/open_vins/blob/master/ov_core/src/init/InertialInitializer.cpp

    // Three axises of the ENU frame in the IMU frame.
    // z-axis
    Eigen::Matrix3d r_IG_;
    Eigen::Matrix3d r_GI_;
    const Eigen::Vector3d &mean_acc_norm = mean_acc.normalized();
    const Eigen::Vector3d &initial_heading_enu_norm = initial_heading_enu.normalized();
    Eigen::Vector3d z_axis = mean_acc_norm;

    // x-axis
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // y-axis
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    Eigen::Matrix3d r_IG;
    r_IG.block<3, 1>(0, 0) = x_axis;
    r_IG.block<3, 1>(0, 1) = y_axis;
    r_IG.block<3, 1>(0, 2) = z_axis;

    r_GI = r_IG.transpose();

    // (1,0,0)을 initial_heading_enu_norm로 회전시키는 행렬 생성
    Eigen::Vector3d v1 = Eigen::Vector3d::UnitX();
    Eigen::Vector3d v2 = initial_heading_enu_norm;

    Eigen::Vector3d axis = v1.cross(v2); 
    double angle = acos(v1.dot(v2));     

    axis.normalize();

    Eigen::Matrix3d K;
    K << 0, -axis.z(), axis.y(),
        axis.z(), 0, -axis.x(),
        -axis.y(), axis.x(), 0;

    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity() + sin(angle) * K + (1 - cos(angle)) * K * K;

    // r_GI에 회전 행렬 곱하기
    r_GI = rotation_matrix * r_GI;

    r_IG = r_GI.transpose();
    r_IG.block<3, 1>(0, 2) = z_axis;
    // x-axis
    x_axis = r_IG.block<3, 1>(0, 0)- z_axis * z_axis.transpose() * r_IG.block<3, 1>(0, 0);
    x_axis.normalize();

    // y-axis
    y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    r_IG.block<3, 1>(0, 0) = x_axis;
    r_IG.block<3, 1>(0, 1) = y_axis;
    r_IG.block<3, 1>(0, 2) = z_axis;
    r_GI = r_IG.transpose();

    return true;
    
}

void FusionNode::publish_save_state() {
    // publish the odometry

    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = frame_id;
    odom_msg.header.stamp = ros::Time::now();

    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.linear() = kf_ptr_->state_ptr_->r_GI;
    T_wb.translation() = kf_ptr_->state_ptr_->p_GI;

    Eigen::Isometry3d T_baselink = Eigen::Isometry3d::Identity();
    T_baselink.linear() = r_imu_2_baselink;
    T_baselink.translation() = t_imu_2_baselink;

    Eigen::Isometry3d T_wb_baselink = T_wb * T_baselink;

    tf::poseEigenToMsg(T_wb_baselink, odom_msg.pose.pose);

    // Transform twist (linear velocity and angular velocity)
    Eigen::Vector3d v_GI = kf_ptr_->state_ptr_->v_GI;
    Eigen::Vector3d av_GI = kf_ptr_->state_ptr_->av_GI;

    // Transform linear velocity considering the IMU's position offset
    Eigen::Vector3d v_baselink = v_GI + av_GI.cross(t_imu_2_baselink);

    // Angular velocity transformation
    Eigen::Vector3d av_baselink = r_imu_2_baselink * av_GI;
    
    tf::vectorEigenToMsg(v_baselink, odom_msg.twist.twist.linear);
    tf::vectorEigenToMsg(av_baselink, odom_msg.twist.twist.angular);
    // tf::vectorEigenToMsg(kf_ptr_->state_ptr_->v_GI, odom_msg.twist.twist.linear);
    Eigen::Matrix3d P_pp = kf_ptr_->state_ptr_->cov.block<3, 3>(0, 0);
    Eigen::Matrix3d P_po = kf_ptr_->state_ptr_->cov.block<3, 3>(0, 6);
    Eigen::Matrix3d P_op = kf_ptr_->state_ptr_->cov.block<3, 3>(6, 0);
    Eigen::Matrix3d P_oo = kf_ptr_->state_ptr_->cov.block<3, 3>(6, 6);
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> P_imu_pose = Eigen::Matrix<double, 6, 6>::Zero();
    P_imu_pose << P_pp, P_po, P_op, P_oo;
    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
    J.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    J.block<3, 3>(3, 3) = r_imu_2_baselink;

    Eigen::Matrix<double, 6, 6> P_baselink_pose = J * P_imu_pose * J.transpose();

    for (int i = 0; i < 36; i++)
        odom_msg.pose.covariance[i] = P_baselink_pose.data()[i];

    // Twist covariance transformation
    Eigen::Matrix3d P_vv = kf_ptr_->state_ptr_->cov.block<3, 3>(3, 3);
    Eigen::Matrix3d P_vo = kf_ptr_->state_ptr_->cov.block<3, 3>(3, 6);
    Eigen::Matrix3d P_ov = kf_ptr_->state_ptr_->cov.block<3, 3>(6, 3);
    Eigen::Matrix3d P_oo_twist = kf_ptr_->state_ptr_->cov.block<3, 3>(6, 6);
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> P_imu_twist = Eigen::Matrix<double, 6, 6>::Zero();
    P_imu_twist << P_vv, P_vo, P_ov, P_oo_twist;

    Eigen::Matrix<double, 6, 6> P_baselink_twist = J * P_imu_twist * J.transpose();

    for (int i = 0; i < 36; i++)
        odom_msg.twist.covariance[i] = P_baselink_twist.data()[i];
    odom_pub_.publish(odom_msg);

    // broadcast tf
    if (pub_tf){
        tf::Transform tf_transform;
        tf_transform.setOrigin(tf::Vector3(T_wb.translation().x(), T_wb.translation().y(), T_wb.translation().z()));
        tf::Quaternion tf_quat;
        tf::Matrix3x3 rot_matrix;
        rot_matrix.setValue(
            T_wb(0, 0), T_wb(0, 1), T_wb(0, 2),
            T_wb(1, 0), T_wb(1, 1), T_wb(1, 2),
            T_wb(2, 0), T_wb(2, 1), T_wb(2, 2)
        );
        rot_matrix.getRotation(tf_quat);
        tf_transform.setRotation(tf_quat);
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf_transform, odom_msg.header.stamp, frame_id, child_frame_id));
    }

    // publish the path
    if (pub_path){
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = odom_msg.header;
        pose_stamped.pose = odom_msg.pose.pose;
        nav_path_.header = pose_stamped.header;
        nav_path_.poses.push_back(pose_stamped);
        path_pub_.publish(nav_path_);
    }
}

}  // namespace ryu

int main(int argc, char **argv) {
    ros::init(argc, argv, "dual_gnss_imu_eskf_node");

    ros::NodeHandle nh("~");
    ryu::FusionNode fusion_node(nh);

    ros::spin();

    return 0;
}
