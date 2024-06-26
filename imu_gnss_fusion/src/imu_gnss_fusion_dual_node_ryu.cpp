#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <Eigen/Core>
#include <deque>
#include <fstream>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include "imu_gnss_fusion/kf.h"

namespace ryu {

class FusionNode {
   public:
    FusionNode(ros::NodeHandle &nh) {
        double acc_n, gyr_n, acc_w, gyr_w;
        double gps_f_x_imuFrame, gps_f_y_imuFrame, gps_f_z_imuFrame;
        double gps_b_x_imuFrame, gps_b_y_imuFrame, gps_b_z_imuFrame;
        double local_long, local_lat, local_alt;
        
        nh.param("acc_noise", acc_n, 1e-2);
        nh.param("gyr_noise", gyr_n, 1e-4);
        nh.param("acc_bias_noise", acc_w, 1e-6);
        nh.param("gyr_bias_noise", gyr_w, 1e-8);

        nh.param("gps_f_x_imuFrame", gps_f_x_imuFrame, 0.);
        nh.param("gps_f_y_imuFrame", gps_f_y_imuFrame, 0.);
        nh.param("gps_f_z_imuFrame", gps_f_z_imuFrame, 0.);
        nh.param("gps_b_x_imuFrame", gps_b_x_imuFrame, 0.);
        nh.param("gps_b_y_imuFrame", gps_b_y_imuFrame, 0.);
        nh.param("gps_b_z_imuFrame", gps_b_z_imuFrame, 0.);
        // gnss position from imu frame
        // This is constant value
        I_p_Gps_f_ = Eigen::Vector3d(gps_f_x_imuFrame, gps_f_y_imuFrame, gps_f_z_imuFrame);
        I_p_Gps_b_ = Eigen::Vector3d(gps_b_x_imuFrame, gps_b_y_imuFrame, gps_b_z_imuFrame);

        nh.param("acc_std_threshold", acc_std_threshold, 3.);
        nh.param("heading_std_threshold", heading_std_threshold, 3.);
        nh.param("delta_gps_threshold", delta_gps_threshold, 3.);

        nh.param("viz_path", viz_path, false);

        // default : UNIST
        nh.param("local_long", local_long, 1e-4);
        nh.param("local_lat", local_lat, 1e-6);
        nh.param("local_alt", local_alt, 1e-8);

        nh.param("pub_tf", pub_tf, false);
        nh.param<std::string>("frame_id", frame_id, "map");
        nh.param<std::string>("child_frame_id", child_frame_id, "gnss_frame");

        std::string topic_imu, topic_gps_front, topic_gps_behind;
        nh.param<std::string>("topic_imu", topic_imu, "/imu/data");
        nh.param<std::string>("topic_gps_front", topic_gps_front, "/gnss_2/llh_position");
        nh.param<std::string>("topic_gps_behind", topic_gps_behind, "/gnss_1/llh_position");

        init_lla_ = Eigen::Vector3d(local_long, local_lat, local_alt);

        kf_ptr_ = std::make_unique<KF>(acc_n, gyr_n, acc_w, gyr_w);



        // ROS sub & pub
        imu_sub_ = nh.subscribe(topic_imu, 10, &FusionNode::imu_callback, this);
        gps_f_sub_ = nh.subscribe(topic_gps_front, 10, &FusionNode::gps_f_callback, this);
        gps_b_sub_ = nh.subscribe(topic_gps_behind, 10, &FusionNode::gps_b_callback, this);

        path_pub_ = nh.advertise<nav_msgs::Path>("nav_path", 10);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("nav_odom", 10);

        // log files
        // file_gps_.open("fusion_gps.csv");
        // file_state_.open("fusion_state.csv");
    }

    ~FusionNode() {
        // file_gps_.close();
        // file_state_.close();
    }

    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
    void gps_f_callback(const sensor_msgs::NavSatFixConstPtr &gps_msg);
    void initialize_heading();
    void gps_b_callback(const sensor_msgs::NavSatFixConstPtr &gps_msg);
    void gps_process(const GpsDataPtr gps_data_ptr, const Eigen::Vector3d& I_p_Gps_);

    bool init_rot_from_imudata(Eigen::Matrix3d &r_GI);

    void publish_save_state();

   private:
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_f_sub_;
    ros::Subscriber gps_b_sub_;
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
    double delta_gps_threshold;
    bool viz_path;

    // init
    bool initialized_rot_ = false;
    bool initialized_gps_ = false;
    bool initialized_heading_ = false;
    const int kImuBufSize = 100;
    const int kGPSBufSize = 5; 
    std::deque<ImuDataConstPtr> imu_buf_;
    std::deque<GpsDataConstPtr> gps_f_buf_;
    std::deque<GpsDataConstPtr> gps_b_buf_;
    ImuDataConstPtr last_imu_ptr_;
    Eigen::Vector3d init_lla_;
    Eigen::Vector3d I_p_Gps_f_;
    Eigen::Vector3d I_p_Gps_b_;
    Eigen::Vector3d initial_heading_enu;


    // KF
    KFPtr kf_ptr_;

    // log files
    // std::ofstream file_gps_;
    // std::ofstream file_state_;
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
    if (!initialized_gps_) {
        imu_buf_.push_back(imu_data_ptr);
        if (imu_buf_.size() > kImuBufSize) imu_buf_.pop_front();
        // return;
    }
    if (!initialized_heading_) {
        if (gps_f_buf_.size() > kGPSBufSize-1 && gps_b_buf_.size() > kGPSBufSize-1 ){
            initialize_heading();
        }
    }

    if (initialized_gps_ && initialized_heading_){
        kf_ptr_->predict(last_imu_ptr_, imu_data_ptr);
        last_imu_ptr_ = imu_data_ptr;
        publish_save_state();
    }

}

void FusionNode::initialize_heading() {
    // gps_f_buf_
    // gps_b_buf_
    Eigen::Vector3d sum_front(0., 0., 0.);
    Eigen::Vector3d sum_behind(0., 0., 0.);

    std::vector<Eigen::Vector3d> gps_f_enu_vec;
    std::vector<Eigen::Vector3d> gps_b_enu_vec;

    for (const auto gps_f : gps_f_buf_) {
        Eigen::Vector3d gps_f_enu; // position of gps from global(map) frame
        ryu::lla2enu(init_lla_, gps_f->lla, &gps_f_enu);
        sum_front += gps_f_enu;
        gps_f_enu_vec.push_back(gps_f_enu);
    }
    for (const auto gps_b : gps_b_buf_) {
        Eigen::Vector3d gps_b_enu; // position of gps from global(map) frame
        ryu::lla2enu(init_lla_, gps_b->lla, &gps_b_enu);
        sum_behind += gps_b_enu;
        gps_b_enu_vec.push_back(gps_b_enu);
    }

    const Eigen::Vector3d mean_front = sum_front / (double)gps_f_buf_.size();
    const Eigen::Vector3d mean_behind = sum_behind / (double)gps_b_buf_.size();

    Eigen::Vector3d sum_err2_f(0., 0., 0.);
    Eigen::Vector3d sum_err2_b(0., 0., 0.);
    for (const auto gps_f_enu : gps_f_enu_vec) sum_err2_f += (gps_f_enu - mean_front).cwiseAbs2();
    for (const auto gps_b_enu : gps_b_enu_vec) sum_err2_b += (gps_b_enu - mean_behind).cwiseAbs2();
    const Eigen::Vector3d std_gps_f = (sum_err2_f / (double)gps_f_buf_.size()).cwiseSqrt();
    const Eigen::Vector3d std_gps_b = (sum_err2_b / (double)gps_b_buf_.size()).cwiseSqrt();

    bool std_flag_f_ = true;
    bool std_flag_b_ = true;
    if (std_gps_f.maxCoeff() > heading_std_threshold) {
        printf("[gnss_imu %s] Too big gps_front_std for heading initialization: (%f, %f, %f)!!!\n", __FUNCTION__, std_gps_f[0], std_gps_f[1], std_gps_f[2]);
        gps_f_buf_.clear();
        std_flag_f_ = false;
    }
    if (std_gps_b.maxCoeff() > heading_std_threshold) {
        printf("[gnss_imu %s] Too big gps_behind_std for heading initialization: (%f, %f, %f)!!!\n", __FUNCTION__, std_gps_b[0], std_gps_b[1], std_gps_b[2]);
        gps_b_buf_.clear();
        std_flag_b_ = false;
    }
    if (!(std_flag_f_ && std_flag_b_)) return;

    double measured_distance = (mean_front - mean_behind).norm();
    double estimated_distance = (I_p_Gps_f_-I_p_Gps_b_).norm();
    double distance_err = std::abs(measured_distance - estimated_distance);

    if (distance_err > delta_gps_threshold){
        printf("[gnss_imu %s] two gps are not located in estimated distance! : %f m !!!\n", __FUNCTION__, distance_err);
        gps_f_buf_.clear();
        gps_b_buf_.clear();
        return;
    }

    initial_heading_enu = mean_front - mean_behind;
    initialized_heading_ = true;
}

void FusionNode::gps_f_callback(const sensor_msgs::NavSatFixConstPtr &gps_msg) {
    // status 2:ground based fix
    // status 0:no fix
    if (gps_msg->status.status != 2) {
        printf("[gnss_imu %s] ERROR: Bad GPS Message!!!\n", __FUNCTION__);
        return;
    }

    GpsDataPtr gps_data_ptr = std::make_shared<GpsData>();
    gps_data_ptr->timestamp = gps_msg->header.stamp.toSec();
    gps_data_ptr->lla[0] = gps_msg->latitude;
    gps_data_ptr->lla[1] = gps_msg->longitude;
    gps_data_ptr->lla[2] = gps_msg->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg->position_covariance.data());

    if (!initialized_heading_) {
        gps_f_buf_.push_back(gps_data_ptr);
        if (gps_f_buf_.size() > kGPSBufSize) gps_f_buf_.pop_front();
        return;
    }
    
    gps_process(gps_data_ptr, I_p_Gps_f_);
}

void FusionNode::gps_b_callback(const sensor_msgs::NavSatFixConstPtr &gps_msg) {
    // status 2:ground based fix
    // status 0:no fix
    if (gps_msg->status.status != 2) {
        printf("[gnss_imu %s] ERROR: Bad GPS Message!!!\n", __FUNCTION__);
        return;
    }

    GpsDataPtr gps_data_ptr = std::make_shared<GpsData>();
    gps_data_ptr->timestamp = gps_msg->header.stamp.toSec();
    gps_data_ptr->lla[0] = gps_msg->latitude;
    gps_data_ptr->lla[1] = gps_msg->longitude;
    gps_data_ptr->lla[2] = gps_msg->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg->position_covariance.data());

    if (!initialized_heading_) {
        gps_b_buf_.push_back(gps_data_ptr);
        if (gps_b_buf_.size() > kGPSBufSize) gps_b_buf_.pop_front();
        return;
    }
    gps_process(gps_data_ptr, I_p_Gps_b_);
}

void FusionNode::gps_process(const GpsDataPtr gps_data_ptr, const Eigen::Vector3d& I_p_Gps_) {
    if (!initialized_rot_) {
        if (imu_buf_.size() < kImuBufSize) {
            printf("[gnss_imu %s] ERROR: Not Enough IMU data for Initialization!!!\n", __FUNCTION__);
            return;
        }

        last_imu_ptr_ = imu_buf_.back();
        if (std::abs(gps_data_ptr->timestamp - last_imu_ptr_->timestamp) > 0.5) {
            printf("[gnss_imu %s] ERROR: Gps and Imu timestamps are not synchronized!!!\n", __FUNCTION__);
            return;
        }

        kf_ptr_->state_ptr_->timestamp = last_imu_ptr_->timestamp;

        if (!init_rot_from_imudata(kf_ptr_->state_ptr_->r_GI)) return;

        // init_lla_ = gps_data_ptr->lla;

        initialized_rot_ = true;

        printf("[gnss_imu %s] System initialized.\n", __FUNCTION__);

        return;
    }

    // convert WGS84 to ENU frame
    Eigen::Vector3d p_G_Gps; // position of gps from global(map) frame
    ryu::lla2enu(init_lla_, gps_data_ptr->lla, &p_G_Gps);

    const Eigen::Vector3d &p_GI = kf_ptr_->state_ptr_->p_GI;
    const Eigen::Matrix3d &r_GI = kf_ptr_->state_ptr_->r_GI;

    // residual
    Eigen::Vector3d residual = p_G_Gps - (p_GI + r_GI * I_p_Gps_);
    double abs_residual = residual.norm();
    std::cout << abs_residual << std::endl;

    // jacobian
    Eigen::Matrix<double, 3, 15> H;
    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(0, 6) = -r_GI * ryu::skew_matrix(I_p_Gps_);

    // measurement covariance
    const Eigen::Matrix3d &V = gps_data_ptr->cov;

    kf_ptr_->update_measurement(H, V, residual);

    if (!initialized_gps_)  initialized_gps_ = true;

    // save gps lla
    // file_gps_ << std::fixed << std::setprecision(15)
    //           << gps_data_ptr->timestamp << ", "
    //           << gps_data_ptr->lla[0] << ", " << gps_data_ptr->lla[1] << ", " << gps_data_ptr->lla[2]
    //           << std::endl;
}

bool FusionNode::init_rot_from_imudata(Eigen::Matrix3d &r_GI) {
    // mean and std of IMU accs
    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (const auto imu_data : imu_buf_) {
        sum_acc += imu_data->acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buf_.size();
    printf("[gnss_imu %s] mean_acc: (%f, %f, %f)!!!\n", __FUNCTION__, mean_acc[0], mean_acc[1], mean_acc[2]);

    // std::cout << (mean_acc.cwiseAbs2()).cwiseSqrt() <<std::endl;
    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (const auto imu_data : imu_buf_) sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buf_.size()).cwiseSqrt();

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
    
    // x-axis
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - mean_acc_norm * mean_acc_norm.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // y-axis
    Eigen::Vector3d y_axis = mean_acc_norm.cross(x_axis);
    y_axis.normalize();

    Eigen::Matrix3d r_IG;
    r_IG.block<3, 1>(0, 0) = x_axis;
    r_IG.block<3, 1>(0, 1) = y_axis;
    r_IG.block<3, 1>(0, 2) = mean_acc_norm;

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
    return true;
    
}

void FusionNode::publish_save_state() {
    // publish the odometry

    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = frame_id;
    odom_msg.header.stamp = ros::Time::now();
    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.linear() = kf_ptr_->state_ptr_->r_GI;
    // std::cout << kf_ptr_->state_ptr_->r_GI << std::endl;
    T_wb.translation() = kf_ptr_->state_ptr_->p_GI;
    tf::poseEigenToMsg(T_wb, odom_msg.pose.pose);
    tf::vectorEigenToMsg(kf_ptr_->state_ptr_->v_GI, odom_msg.twist.twist.linear);
    Eigen::Matrix3d P_pp = kf_ptr_->state_ptr_->cov.block<3, 3>(0, 0);
    Eigen::Matrix3d P_po = kf_ptr_->state_ptr_->cov.block<3, 3>(0, 6);
    Eigen::Matrix3d P_op = kf_ptr_->state_ptr_->cov.block<3, 3>(6, 0);
    Eigen::Matrix3d P_oo = kf_ptr_->state_ptr_->cov.block<3, 3>(6, 6);
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> P_imu_pose = Eigen::Matrix<double, 6, 6>::Zero();
    P_imu_pose << P_pp, P_po, P_op, P_oo;
    for (int i = 0; i < 36; i++)
        odom_msg.pose.covariance[i] = P_imu_pose.data()[i];
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
    if (viz_path){
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = odom_msg.header;
        pose_stamped.pose = odom_msg.pose.pose;
        nav_path_.header = pose_stamped.header;
        nav_path_.poses.push_back(pose_stamped);
        path_pub_.publish(nav_path_);
    }

    // save state p q lla
    // std::shared_ptr<KF::State> kf_state(kf_ptr_->state_ptr_);
    // Eigen::Vector3d lla;
    // ryu::enu2lla(init_lla_, kf_state->p_GI, &lla);  // convert ENU state to lla
    // const Eigen::Quaterniond q_GI(kf_state->r_GI);
    // file_state_ << std::fixed << std::setprecision(15)
    //             << kf_state->timestamp << ", "
    //             << kf_state->p_GI[0] << ", " << kf_state->p_GI[1] << ", " << kf_state->p_GI[2] << ", "
    //             << q_GI.x() << ", " << q_GI.y() << ", " << q_GI.z() << ", " << q_GI.w() << ", "
    //             << lla[0] << ", " << lla[1] << ", " << lla[2]
    //             << std::endl;
}

}  // namespace ryu

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_gnss_fusion");

    ros::NodeHandle nh;
    ryu::FusionNode fusion_node(nh);

    ros::spin();

    return 0;
}
