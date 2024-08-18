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
#include "kf.h"
#include "Gnss.h"
#include "Imu.h"

namespace ryu {

class FusionNode {
   public:
    FusionNode(ros::NodeHandle &nh) {
        //////////////////ESKF///////////////////////
        nh.param("consider_delay", consider_delay, false);
        nh.param("kDelayImuBufSize", kDelayImuBufSize, 20);
        if(!consider_delay) kDelayImuBufSize=0;
        kImuBufSize = kDelayImuBufSize + 1;
        nh.param("use_dual_gnss", use_dual_gnss, false);
        std::string dual_gnss_mode;
        if(use_dual_gnss){
            nh.param<std::string>("dual_gnss_mode", dual_gnss_mode, "only_initialization");
            if(dual_gnss_mode == "6DOF_update"){
                kf_update_dof = 6;
            }
        } 
        //////////////////GNSS///////////////////////
        /////////////////Origin//////////////////////
        double origin_long, origin_lat, origin_alt;
        nh.param("origin_long", origin_long, 35.571127);
        nh.param("origin_lat", origin_lat, 129.1874088);
        nh.param("origin_alt", origin_alt, 76.716);
        origin_enu_ = Eigen::Vector3d(origin_long, origin_lat, origin_alt);

        std::string topic_gnss_main, topic_gnss_sub;
        double gnss_main_x_imuFrame, gnss_main_y_imuFrame, gnss_main_z_imuFrame;
        double gnss_sub_x_imuFrame, gnss_sub_y_imuFrame, gnss_sub_z_imuFrame;
        nh.param<std::string>("topic_gnss_main", topic_gnss_main, "/gnss_2/llh_position");
        nh.param<std::string>("topic_gnss_sub", topic_gnss_sub, "/gnss_1/llh_position");
        nh.param("gnss_main_x_imuFrame", gnss_main_x_imuFrame, 0.);
        nh.param("gnss_main_y_imuFrame", gnss_main_y_imuFrame, 0.);
        nh.param("gnss_main_z_imuFrame", gnss_main_z_imuFrame, 0.);
        nh.param("gnss_sub_x_imuFrame", gnss_sub_x_imuFrame, 1.47);
        nh.param("gnss_sub_y_imuFrame", gnss_sub_y_imuFrame, 0.);
        nh.param("gnss_sub_z_imuFrame", gnss_sub_z_imuFrame, 0.);
        gnss_sub_pos_imuFrame = Eigen::Vector3d(gnss_sub_x_imuFrame, gnss_sub_y_imuFrame, gnss_sub_z_imuFrame);
        gnss_main_pos_imuFrame = Eigen::Vector3d(gnss_main_x_imuFrame, gnss_main_y_imuFrame, gnss_main_z_imuFrame);

        //////////////////IMU///////////////////////        
        std::string topic_imu;
        nh.param<std::string>("topic_imu", topic_imu, "/imu/data");
        double acc_n, gyr_n, acc_w, gyr_w;
        nh.param("acc_noise", acc_n, 1e-2);
        nh.param("gyr_noise", gyr_n, 1e-4);
        nh.param("acc_bias_noise", acc_w, 1e-6);
        nh.param("gyr_bias_noise", gyr_w, 1e-8);

        ///////////////INITIALIZATION////////////////
        nh.param("kInitGnssBufSize", kInitGnssBufSize, 5);
        nh.param("kInitImuBufSize", kInitImuBufSize, 100);
        nh.param("kAccStdThreshold", kAccStdThreshold, 3.);
        nh.param("kGnssPosStdThreshold", kGnssPosStdThreshold, 3.);
        nh.param("kGnssPosDiffThreshold", kGnssPosDiffThreshold, 0.05);
        nh.param("kTimeSyncThreshold", kTimeSyncThreshold, 0.05);
        
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
        orientation_baselink_from_Imu = Eigen::Quaterniond(baes_link_qw_imuFrame, baes_link_qx_imuFrame, baes_link_qy_imuFrame, baes_link_qz_imuFrame).toRotationMatrix();
        position_baselink_from_Imu = Eigen::Vector3d(baes_link_x_imuFrame, baes_link_y_imuFrame, baes_link_z_imuFrame);
        
        ////////////////// TF ///////////////////////  
        nh.param("pub_tf", pub_tf, false);
        nh.param<std::string>("frame_id", frame_id, "map");
        nh.param<std::string>("child_frame_id", child_frame_id, "base_link");

        ////////////////// OTHERS ///////////////////////      
        nh.param("pub_path", pub_path, false);
        std::string topic_path;
        nh.param<std::string>("topic_path", topic_path, "/eskf_path");

        ////////////////// KF ///////////////////////  
        if(use_dual_gnss){
            kf_ptr_ = std::make_unique<KF>(acc_n, gyr_n, acc_w, gyr_w, 0.01, 0.01, 10.);
            delay_kf_ptr_ = std::make_unique<KF>(acc_n, gyr_n, acc_w, gyr_w, 0.01, 0.01, 10.);
        }
        else{
            kf_ptr_ = std::make_unique<KF>(acc_n, gyr_n, acc_w, gyr_w, 10., 10., 100.);
            delay_kf_ptr_ = std::make_unique<KF>(acc_n, gyr_n, acc_w, gyr_w, 10., 10., 100.);
        }

        double estimated_distance_btw_gnss = (gnss_sub_pos_imuFrame - gnss_main_pos_imuFrame).norm();
        gnss_preprocess_ptr_ = std::make_unique<Gnss>(origin_enu_, use_dual_gnss, kInitGnssBufSize, kGnssPosStdThreshold, estimated_distance_btw_gnss, kGnssPosDiffThreshold);
        imu_preprocess_ptr_ = std::make_unique<Imu>(kInitImuBufSize, kAccStdThreshold);

        ////////////////// ROS Sub && Pub///////////////////////  
        imu_sub_ = nh.subscribe(topic_imu, 10, &FusionNode::imu_callback, this);
        gnss_main_sub_ = nh.subscribe(topic_gnss_sub, 10, &FusionNode::gnss_main_callback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>(topic_odometry, 10);

        if(use_dual_gnss) gnss_sub_sub_ = nh.subscribe(topic_gnss_main, 10, &FusionNode::gnss_sub_callback, this);
        if(pub_path) path_pub_ = nh.advertise<nav_msgs::Path>(topic_path, 10);
    }

    ~FusionNode() {
    }

    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
    void gnss_main_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg);
    void gnss_sub_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg);

    void compute_initial_orientation(const Eigen::Vector3d globalZaxis_imuFrame, const Eigen::Vector3d vector_global, const Eigen::Vector3d vector_imu);
    void compute_initial_position(const Eigen::Vector3d intial_position_main);
    void gnss_to_imu_position(Eigen::Vector3d& imu_position_global, const Eigen::Vector3d& gnss_position_global, const Eigen::Vector3d& gnss_position_imu);
    bool measure_imu_orientation(const gnssDataPtr& gnss_ptr, Eigen::Vector3d& imu_position_global, Eigen::Matrix3d& imu_orientation_global,\
                                    StatePtr state_ptr_, Eigen::Matrix3d& imu_orientation_residual, bool is_sub);
    void publish_save_state();

   private:
    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_sub_;
    ros::Subscriber gnss_main_sub_;
    ros::Publisher path_pub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    nav_msgs::Path nav_path_;
    // rosparam
    std::string frame_id;
    std::string child_frame_id;
    bool pub_tf;
    
    //////////////////ESKF///////////////////////
    bool consider_delay;
    bool use_dual_gnss;

    ///////////////INITIALIZATION////////////////
    double kAccStdThreshold;
    double kGnssPosStdThreshold;
    double kGnssPosDiffThreshold;
    double kTimeSyncThreshold;
    bool pub_path;

    double cur_gnss_time, prev_gnss_time;
    double cur_del_gnss_time, prev_del_gnss_time;
    bool init_prev_gnss_time, init_prev_del_gnss_time;
    Eigen::Vector3d gnss_main_, gnss_sub_;

    // init
    int kInitImuBufSize;
    int kDelayImuBufSize;
    int kImuBufSize;
    int kInitGnssBufSize; 
    std::deque<ImuDataConstPtr> imu_buf_;
    std::deque<ImuDataConstPtr> delay_imu_buf_;
    std::deque<StatePtr> delay_state_buf_;
    ImuDataConstPtr last_imu_ptr_;
    Eigen::Vector3d origin_enu_;
    Eigen::Vector3d gnss_sub_pos_imuFrame;
    Eigen::Vector3d gnss_main_pos_imuFrame;
    Eigen::Vector3d initial_heading_enu;

    bool imu_start_flag = false;
    bool init_end_flag = false;
    bool is_initialized_orientation = false;
    bool is_initialized_position = false;
    double init_start;
    double init_end;

    // imu to base_link
    Eigen::Matrix3d orientation_baselink_from_Imu;
    Eigen::Vector3d position_baselink_from_Imu;

    // KF
    KFPtr kf_ptr_;
    KFPtr delay_kf_ptr_;
    int kf_update_dof=3;

    GnssPtr gnss_preprocess_ptr_;
    ImuPtr imu_preprocess_ptr_;
};

void FusionNode::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
    ImuDataPtr imu_data_ptr = std::make_shared<ImuData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
    imu_data_ptr->acc[1] = imu_msg->linear_acceleration.y;
    imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
    imu_data_ptr->gyr[0] = imu_msg->angular_velocity.x;
    imu_data_ptr->gyr[1] = imu_msg->angular_velocity.y;
    imu_data_ptr->gyr[2] = imu_msg->angular_velocity.z;
    if (!imu_start_flag) {
        init_start = imu_msg->header.stamp.toSec();
        imu_start_flag = true;
    }
    if(!(is_initialized_orientation && is_initialized_position)){
        imu_preprocess_ptr_->initializer(imu_data_ptr);
        if(!(imu_preprocess_ptr_->is_initialized && gnss_preprocess_ptr_->is_initialized)){
            last_imu_ptr_ = imu_data_ptr;
            return;
        }
        else{
            if(use_dual_gnss){
                Eigen::Vector3d gnss_main_to_sub_imuFrame = gnss_sub_pos_imuFrame - gnss_main_pos_imuFrame;
                compute_initial_orientation(imu_preprocess_ptr_->globalZaxis_imuFrame, gnss_preprocess_ptr_->gnss_main_to_sub_global, gnss_main_to_sub_imuFrame);
            }
            else{
                compute_initial_orientation(imu_preprocess_ptr_->globalZaxis_imuFrame, Eigen::Vector3d(1,0,0), Eigen::Vector3d(1,0,0));
            }
            compute_initial_position(gnss_preprocess_ptr_->intial_position_main);
        }
    }
    if(!(is_initialized_orientation && is_initialized_position)){
        last_imu_ptr_ = imu_data_ptr;
        return;
    }

    if (is_initialized_orientation && is_initialized_position && !init_end_flag){
        init_end = imu_msg->header.stamp.toSec();
        double initialization_time = init_end - init_start;
        printf("[gnss_imu %s] Initialization takes %f secs!!!!\n", __FUNCTION__, initialization_time);
        init_end_flag = true;
    }
    delay_imu_buf_.push_back(last_imu_ptr_);
    delay_state_buf_.push_back(std::make_shared<State>(*kf_ptr_->state_ptr_));
    if (delay_imu_buf_.size() > kImuBufSize) delay_imu_buf_.pop_front();
    if (delay_state_buf_.size() > kImuBufSize) delay_state_buf_.pop_front();
    kf_ptr_->predict(last_imu_ptr_, imu_data_ptr);
    last_imu_ptr_ = imu_data_ptr;
    publish_save_state();
}

void FusionNode::compute_initial_orientation(const Eigen::Vector3d globalZaxis_imuFrame, const Eigen::Vector3d vector_global, const Eigen::Vector3d vector_imu) {
    Eigen::Vector3d x_axis, y_axis, z_axis;
    const Eigen::Vector3d vector_global_norm = vector_global.normalized();
    const Eigen::Vector3d vector_imu_norm = vector_imu.normalized();
    z_axis = globalZaxis_imuFrame;
    double a, b, c, d, e, f, g, h, i;
    // double x, y, z;
    a = z_axis(0);
    b = z_axis(1);
    c = z_axis(2);
    d = vector_imu_norm(0);
    e = vector_imu_norm(1);
    f = vector_imu_norm(2);
    g = vector_global_norm(0);
    h = vector_global_norm(1);
    i = vector_global_norm(2);
    // R is imu to global rotation
    // R*globalZaxis_imuFrame = (0,0,1)

    //     [  x     y     z  ] 
    // R = [bz-cy cx-az ay-bx]
    //     [  a     b     c  ]
    // (1)       ax +       by +       cz = 0
    // R*vector_imu_norm(d,e,f) = vector_global_norm(g,h,i)
    // (2)       dx +       ey +       fz = g
    // (3) (ce-bf)x + (af-cd)y + (bd-ae)z = h
    Eigen::Matrix3d A; 
    Eigen::Vector3d B; 
    A <<        a,         b,        c,
                d,         e,        f,
        (e*c-f*b), (f*a-d*c), (d*b-e*a);
    B = Eigen::Vector3d(0., g, h);

    // Perform Singular Value Decomposition (SVD)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::Vector3d solution = svd.solve(B);
    double x = solution(0);
    double y = solution(1);
    double z = solution(2);

    Eigen::Matrix3d initial_imu_orientation_Global;
    initial_imu_orientation_Global.block<1, 3>(0, 0) = Eigen::Vector3d(x, y, z);
    initial_imu_orientation_Global.block<1, 3>(1, 0) = Eigen::Vector3d(b*z-c*y, c*x-a*z, a*y-b*x);;
    initial_imu_orientation_Global.block<1, 3>(2, 0) = z_axis;
    kf_ptr_->state_ptr_->r_GI = initial_imu_orientation_Global;
    
    is_initialized_orientation = true;
    return;
}

void FusionNode::compute_initial_position(const Eigen::Vector3d intial_gnss_position_main) {
    Eigen::Matrix3d orientation_imu_global = kf_ptr_->state_ptr_->r_GI;
    Eigen::Vector3d initial_imu_position_global = intial_gnss_position_main - orientation_imu_global * gnss_main_pos_imuFrame;
    kf_ptr_->state_ptr_->p_GI = initial_imu_position_global;
    is_initialized_position = true;
    return;
}

void FusionNode::gnss_to_imu_position(Eigen::Vector3d& imu_position_global, const Eigen::Vector3d& gnss_position_global, const Eigen::Vector3d& gnss_position_imu) {
    const Eigen::Matrix3d imu_orientation = kf_ptr_->state_ptr_->r_GI;

    imu_position_global = gnss_position_global - imu_orientation * gnss_position_imu;
}

bool FusionNode::measure_imu_orientation(const gnssDataPtr& gnss_ptr, Eigen::Vector3d& imu_position_global, Eigen::Matrix3d& imu_orientation_global,\
                                            StatePtr state_ptr_, Eigen::Matrix3d& imu_orientation_residual, bool is_sub) {
    bool estimate_success = false;
    cur_gnss_time = gnss_ptr -> timestamp;
    if(is_sub){
        gnss_sub_ = gnss_ptr -> xyz_enu;
    }
    else{
        gnss_main_ = gnss_ptr -> xyz_enu;
    }
    if(!init_prev_gnss_time){
        prev_gnss_time = cur_gnss_time;
        init_prev_gnss_time = true;
        return false;
    }
    if(!init_prev_del_gnss_time){
        prev_del_gnss_time = cur_gnss_time - prev_gnss_time;
        init_prev_del_gnss_time = true;
        prev_gnss_time = cur_gnss_time;
        return false;
    }
    cur_del_gnss_time = cur_gnss_time - prev_gnss_time; 


    if(prev_del_gnss_time > cur_del_gnss_time){
        Eigen::Vector3d measured_main_to_sub, predicted_main_to_sub;
        Eigen::Matrix3d residual_orientation, measured_orientation;
        
        measured_main_to_sub = gnss_sub_ - gnss_main_;
        predicted_main_to_sub = state_ptr_->r_GI*(gnss_sub_pos_imuFrame - gnss_main_pos_imuFrame);

        Eigen::Vector3d rotation_axis = predicted_main_to_sub.cross(measured_main_to_sub);
        double rotation_angle = acos(measured_main_to_sub.normalized().dot(predicted_main_to_sub.normalized()));
        Eigen::AngleAxisd residual_rotation(rotation_angle, rotation_axis.normalized());

        residual_orientation = residual_rotation.toRotationMatrix();
        measured_orientation = state_ptr_->r_GI * residual_orientation;
        imu_position_global = gnss_main_ - state_ptr_->r_GI*gnss_main_pos_imuFrame;
        // std::cout<<"measured_imu_position"<<std::endl;
        // std::cout<<gnss_main_<<std::endl;
        // std::cout<<kf_ptr_->state_ptr_->r_GI<<std::endl;
        // std::cout<<gnss_main_pos_imuFrame<<std::endl;
        // imu_position_global = gnss_main_ - measured_orientation*gnss_main_pos_imuFrame;
        imu_orientation_residual = residual_orientation;
        estimate_success = true;
        // double angle1 = acos(measured_main_to_sub.normalized().dot((predicted_main_to_sub).normalized()));
        // double angle2 = acos(measured_main_to_sub.normalized().dot((measured_orientation*(gnss_sub_pos_imuFrame - gnss_main_pos_imuFrame)).normalized()));
        // std::cout << "!!!!!!!!!"<<std::endl;
        // if (angle1 > angle2) std::cout << "work good!!!!!!"<<std::endl;
        // else std::cout << "work bad.........."<<std::endl;
        // std::cout << angle1<<std::endl;
        // std::cout << angle2<<std::endl;
    }
    prev_gnss_time = cur_gnss_time;
    prev_del_gnss_time = cur_del_gnss_time;
    if(estimate_success){
        return true;
    }
    else{
        return false;
    } 
}   


void FusionNode::gnss_main_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg) {
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
    ryu::lla2enu(origin_enu_, gnss_data_ptr->lla, &gnss_data_ptr->xyz_enu);

    gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gnss_msg->position_covariance.data());

    if (std::abs(gnss_data_ptr->timestamp - last_imu_ptr_->timestamp) > kTimeSyncThreshold) {
        printf("[gnss_imu %s] ERROR: gnss and Imu timestamps are not synchronized!!!\n", __FUNCTION__);
        return;
    }

    bool is_sub = false;
    if(!(is_initialized_orientation && is_initialized_position)){
        gnss_preprocess_ptr_->initializer(gnss_data_ptr, is_sub);
        if(use_dual_gnss) gnss_preprocess_ptr_->computeHeadingDualGnss();
        else gnss_preprocess_ptr_->initializeSingleGnss();
        return;
    }

    Eigen::Vector3d measured_imu_position;
    gnss_to_imu_position(measured_imu_position, gnss_data_ptr->xyz_enu, gnss_main_pos_imuFrame);


    if (!use_dual_gnss){
        if (delay_imu_buf_.size() > kImuBufSize){
            StatePtr first_state_ptr = delay_state_buf_.front();
            delay_kf_ptr_->state_ptr_ = first_state_ptr;
            delay_kf_ptr_->update_3DOF(measured_imu_position, gnss_data_ptr->cov, gnss_main_pos_imuFrame);

            for (size_t i = 1; i < delay_imu_buf_.size(); ++i) {
                delay_kf_ptr_->predict(delay_imu_buf_[i-1], delay_imu_buf_[i]);
            }
            // kf_ptr_ -> update_6DOF(delay_kf_ptr_->state_ptr_->p_GI, delay_kf_ptr_->state_ptr_->r_GI, residual_imu_orientaion, gnss_data_ptr->cov, gnss_main_pos_imuFrame, gnss_orientation_imuFrame);
            // std::cout <<first_state_ptr->p_GI << std::endl;
            // std::cout <<delay_kf_ptr_->state_ptr_->p_GI << std::endl;
            kf_ptr_->state_ptr_ = delay_kf_ptr_->state_ptr_;
            delay_imu_buf_.clear();
        }
    } 
    else{
        Eigen::Matrix3d measured_imu_orientation, residual_imu_orientaion, gnss_orientation_imuFrame;
        gnss_orientation_imuFrame.setIdentity();
        if (delay_imu_buf_.size() >= kImuBufSize){
            StatePtr first_state_ptr = delay_state_buf_.front();
            delay_kf_ptr_->state_ptr_ = first_state_ptr;
            if(measure_imu_orientation(gnss_data_ptr, measured_imu_position, measured_imu_orientation, first_state_ptr, residual_imu_orientaion, is_sub)){
                if(kf_update_dof==3){
                delay_kf_ptr_->update_3DOF(measured_imu_position, gnss_data_ptr->cov, gnss_main_pos_imuFrame);
                }
                else if(kf_update_dof==6){
                delay_kf_ptr_ -> update_6DOF(measured_imu_position, measured_imu_orientation, residual_imu_orientaion, gnss_data_ptr->cov, gnss_main_pos_imuFrame, gnss_orientation_imuFrame);
                }
                for (size_t i = 1; i < delay_imu_buf_.size(); ++i) {
                    delay_kf_ptr_->predict(delay_imu_buf_[i-1], delay_imu_buf_[i]);
                }
                // kf_ptr_ -> update_6DOF(delay_kf_ptr_->state_ptr_->p_GI, delay_kf_ptr_->state_ptr_->r_GI, residual_imu_orientaion, gnss_data_ptr->cov, gnss_main_pos_imuFrame, gnss_orientation_imuFrame);
                // std::cout <<first_state_ptr->p_GI << std::endl;
                // std::cout <<delay_kf_ptr_->state_ptr_->p_GI << std::endl;
                kf_ptr_->state_ptr_ = delay_kf_ptr_->state_ptr_;
                delay_imu_buf_.clear();        
                }
            }
        }
}

void FusionNode::gnss_sub_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg) {
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
    ryu::lla2enu(origin_enu_, gnss_data_ptr->lla, &gnss_data_ptr->xyz_enu);

    gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gnss_msg->position_covariance.data());

    if (std::abs(gnss_data_ptr->timestamp - last_imu_ptr_->timestamp) > kTimeSyncThreshold) {
        printf("[gnss_imu %s] ERROR: gnss and Imu timestamps are not synchronized!!!\n", __FUNCTION__);
        return;
    }

    bool is_sub = true;
    if(!(is_initialized_orientation && is_initialized_position)){
        gnss_preprocess_ptr_->initializer(gnss_data_ptr, is_sub);
        gnss_preprocess_ptr_->computeHeadingDualGnss();
        return;
    }    

    Eigen::Vector3d measured_imu_position;
    Eigen::Matrix3d measured_imu_orientation, residual_imu_orientaion, gnss_orientation_imuFrame;
        gnss_orientation_imuFrame.setIdentity();
        if (delay_imu_buf_.size() >= kImuBufSize){
            StatePtr first_state_ptr = delay_state_buf_.front();
            delay_kf_ptr_->state_ptr_ = first_state_ptr;
            if(measure_imu_orientation(gnss_data_ptr, measured_imu_position, measured_imu_orientation, first_state_ptr, residual_imu_orientaion, is_sub)){
                if(kf_update_dof==3){
                delay_kf_ptr_->update_3DOF(measured_imu_position, gnss_data_ptr->cov, gnss_main_pos_imuFrame);
                }
                else if(kf_update_dof==6){
                delay_kf_ptr_ -> update_6DOF(measured_imu_position, measured_imu_orientation, residual_imu_orientaion, gnss_data_ptr->cov, gnss_main_pos_imuFrame, gnss_orientation_imuFrame);
                }        
                for (size_t i = 1; i < delay_imu_buf_.size(); ++i) {
                    delay_kf_ptr_->predict(delay_imu_buf_[i-1], delay_imu_buf_[i]);
                }
                // kf_ptr_ -> update_6DOF(delay_kf_ptr_->state_ptr_->p_GI, delay_kf_ptr_->state_ptr_->r_GI, residual_imu_orientaion, gnss_data_ptr->cov, gnss_main_pos_imuFrame, gnss_orientation_imuFrame);
                // std::cout <<first_state_ptr->p_GI << std::endl;
                // std::cout <<delay_kf_ptr_->state_ptr_->p_GI << std::endl;
                kf_ptr_->state_ptr_ = delay_kf_ptr_->state_ptr_;
                delay_imu_buf_.clear();   
            }
        }
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
    T_baselink.linear() = orientation_baselink_from_Imu;
    T_baselink.translation() = position_baselink_from_Imu;

    Eigen::Isometry3d T_wb_baselink = T_wb * T_baselink;

    tf::poseEigenToMsg(T_wb_baselink, odom_msg.pose.pose);

    // Transform twist (linear velocity and angular velocity)
    Eigen::Vector3d v_GI = kf_ptr_->state_ptr_->v_GI;
    Eigen::Vector3d av_GI = kf_ptr_->state_ptr_->av_GI;

    // Transform linear velocity considering the IMU's position offset
    Eigen::Vector3d v_baselink = v_GI + av_GI.cross(position_baselink_from_Imu);

    // Angular velocity transformation
    Eigen::Vector3d av_baselink = orientation_baselink_from_Imu * av_GI;
    
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
    J.block<3, 3>(3, 3) = orientation_baselink_from_Imu;

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
