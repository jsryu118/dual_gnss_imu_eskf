#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <deque>
#include <fstream>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <microstrain_inertial_msgs/HumanReadableStatus.h>
#include <future>
#include "kf.h"

namespace ryu {

class SwitchingNode {
public:
    SwitchingNode(ros::NodeHandle &nh) {
        std::string topic_status, topic_manual_switching_signal, topic_gq7, topic_eskf, topic_pubodom, topic_pubpose, topic_pubvel, log_path;
        nh.param<std::string>("topic_status", topic_status, "/ekf/status");
        nh.param<std::string>("topic_manual_switching_signal", topic_manual_switching_signal, "/manual_switching_signal");
        nh.param<std::string>("topic_gq7", topic_gq7, "/ekf/odometry_map");
        nh.param<std::string>("topic_eskf", topic_eskf, "/eskf/odom");
        nh.param<std::string>("topic_pubodom", topic_pubodom, "/switching_result");
        nh.param<std::string>("topic_pubpose", topic_pubpose, "/current_pose");
        nh.param<std::string>("topic_pubvel", topic_pubvel, "/current_velocity");
        nh.param<std::string>("log_path", log_path, "odom_log.txt");
        bool use_lidar_odom;
        nh.param("use_lidar_odom", use_lidar_odom, false);

        // ROS sub & pub
        status_sub_ = nh.subscribe(topic_status, 10, &SwitchingNode::status_callback, this);
        manualsignal_sub_ = nh.subscribe(topic_manual_switching_signal, 10, &SwitchingNode::manualsignal_callback, this);
        gq7_sub_ = nh.subscribe(topic_gq7, 10, &SwitchingNode::gq7_callback, this);
        eskf_sub_ = nh.subscribe(topic_eskf, 10, &SwitchingNode::eskf_callback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>(topic_pubodom, 10);

        ////////////////// KF ///////////////////////  
        double pos_n, vel_n;
        nh.param("pos_n", pos_n, 0.001);
        nh.param("vel_n", vel_n, 0.001);
        kf_ptr_ = std::make_unique<KF>(pos_n, vel_n);

        if (use_lidar_odom){
            topic_pubpose += "_gnss"; 
            topic_pubvel += "_gnss";
        }
        pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(topic_pubpose, 10);
        vel_pub_ = nh.advertise<geometry_msgs::TwistStamped >(topic_pubvel, 10);

        if (std::ifstream(log_path)) {
            // If the file exists, delete it
            if (std::remove(log_path.c_str()) != 0) {
                ROS_ERROR("Failed to delete existing log file!");
            }
        }
        log_file_.open(log_path, std::ios::out | std::ios::app);
        if (!log_file_.is_open()) {
            ROS_ERROR("Failed to open log file!");
        }
        ////////////////// TF ///////////////////////  
        nh.param("pub_tf", pub_tf, false);
        nh.param<std::string>("frame_id", frame_id, "map");
        nh.param<std::string>("child_frame_id", child_frame_id, "base_link");
    }

    ~SwitchingNode() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

    void status_callback(const microstrain_inertial_msgs::HumanReadableStatusConstPtr &status_msg);
    void manualsignal_callback(const std_msgs::Bool &manualsignal_msg);
    void state_publisher(const nav_msgs::OdometryConstPtr &odom_msg);
    void gq7_callback(const nav_msgs::OdometryConstPtr &gq7_msg);
    void eskf_callback(const nav_msgs::OdometryConstPtr &eskf_msg);
    void logger(bool stable_flag);
    void logger_single(int state_flag, const nav_msgs::OdometryConstPtr &odom_msg);
    void prepare_kf(const nav_msgs::OdometryConstPtr& odom_msg);
    nav_msgs::Odometry apply_kf(const nav_msgs::OdometryConstPtr& odom_msg);

private:
    ros::Subscriber status_sub_;
    ros::Subscriber manualsignal_sub_;
    ros::Subscriber gq7_sub_;
    ros::Subscriber eskf_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher vel_pub_;
    tf::TransformBroadcaster tf_broadcaster_;

    bool is_initialized_gq7_ = false;
    bool is_initialized_eskf_ = false;
    bool is_stable_ = false;
    bool is_stable_switching_ = true;

    nav_msgs::Odometry gq7_odom;
    nav_msgs::Odometry eskf_odom;
    std::ofstream log_file_;
    int num_switching = 0;
    int num_switching_threshold = 300; // 100hz x 3sec

    double switching_stable_threshold = 0.01; // m^2
    KFPtr kf_ptr_;
    double last_t;
    bool pub_tf;
    std::string frame_id, child_frame_id;
    std::deque<nav_msgs::OdometryConstPtr> gq7_odom_buf_;
    std::deque<nav_msgs::OdometryConstPtr> eskf_odom_buf_;
    bool is_stable_before_ = false;
};

void SwitchingNode::prepare_kf(const nav_msgs::OdometryConstPtr& odom_msg) {
    auto& state = kf_ptr_->state_ptr_->x;
    last_t = odom_msg->header.stamp.toSec();
    state(0) = odom_msg->pose.pose.position.x;
    state(1) = odom_msg->pose.pose.position.y;
    state(2) = odom_msg->pose.pose.position.z;
    state(3) = odom_msg->twist.twist.linear.x;
    state(4) = odom_msg->twist.twist.linear.y;
    state(5) = odom_msg->twist.twist.linear.z;
}

nav_msgs::Odometry SwitchingNode::apply_kf(const nav_msgs::OdometryConstPtr& odom_msg) {
    double dt = odom_msg->header.stamp.toSec() - last_t;
    if (dt < DBL_EPSILON) dt = 0.0001;
    Eigen::Matrix<double, kStateDim, kStateDim> F = Eigen::Matrix<double, kStateDim, kStateDim>::Identity();
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt; // Position to velocity

    // 잡음 공분산 행렬 Q
    Eigen::Matrix<double, kStateDim, kStateDim> Q = Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
    Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (0.01 * dt * dt); // Position noise
    Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (0.01 * dt); // Velocity noise

    kf_ptr_->predict(F, Q);

    Eigen::Matrix<double, 6, kStateDim> H;
    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // Position
    H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();  // Velocity

    Eigen::Matrix<double, 6, 6> R;
    R.setIdentity();
    R.block<3, 3>(0, 0) *= 0.01;  // Position noise
    R.block<3, 3>(3, 3) *= 0.01;  // Velocity noise

    Eigen::Matrix<double, 6, 1> z;
    z.block<3, 1>(0, 0) = Eigen::Vector3d(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    z.block<3, 1>(3, 0) = Eigen::Vector3d(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.z);

    kf_ptr_->update(H, R, z);
    const auto& state = kf_ptr_->state_ptr_->x;
    nav_msgs::Odometry kf_odom_msg = *odom_msg;
    kf_odom_msg.pose.pose.position.x = state(0);
    kf_odom_msg.pose.pose.position.y = state(1);
    kf_odom_msg.pose.pose.position.z = state(2);
    kf_odom_msg.twist.twist.linear.x = state(3);
    kf_odom_msg.twist.twist.linear.y = state(4);
    kf_odom_msg.twist.twist.linear.z = state(5);
    last_t = kf_odom_msg.header.stamp.toSec();

    // Calculate the difference between state and odom_msg
    double dx = state(0) - odom_msg->pose.pose.position.x;
    double dy = state(1) - odom_msg->pose.pose.position.y;
    double dz = state(2) - odom_msg->pose.pose.position.z;

    // Calculate the parameter as the squared sum of differences
    double param = dx*dx + dy*dy + dz*dz;
    if (param < switching_stable_threshold){
        is_stable_switching_ = true;
    }
    return kf_odom_msg;
}

void SwitchingNode::logger(bool stable_flag) {
    if (log_file_.is_open()) {
        if (stable_flag) {
            log_file_ << stable_flag << " "
                      << gq7_odom.pose.pose.position.x << " "
                      << gq7_odom.pose.pose.position.y << " "
                      << gq7_odom.pose.pose.position.z << std::endl;
        } else {
            if (is_initialized_eskf_){
            log_file_ << stable_flag << " "
                      << eskf_odom.pose.pose.position.x << " "
                      << eskf_odom.pose.pose.position.y << " "
                      << eskf_odom.pose.pose.position.z << std::endl;
            }
        }
    }
}

void SwitchingNode::logger_single(int state_flag, const nav_msgs::OdometryConstPtr &odom_msg) {
    if (log_file_.is_open()) {
        log_file_ << state_flag << " "
                << odom_msg->pose.pose.position.x << " "
                << odom_msg->pose.pose.position.y << " "
                << odom_msg->pose.pose.position.z << std::endl;
    }
}

void SwitchingNode::status_callback(const microstrain_inertial_msgs::HumanReadableStatusConstPtr &status_msg) {}

void SwitchingNode::manualsignal_callback(const std_msgs::Bool &manualsignal_msg){
    is_stable_before_ = is_stable_;

    if (manualsignal_msg.data){
        is_stable_= true;
        is_stable_switching_ = false;

        logger(is_stable_);
    } else {
        is_stable_= false;
        is_stable_switching_ = false;

        logger(is_stable_);
    }

    if(is_stable_before_ && !is_stable_) {
        // 비동기 호출
        auto gq7_buf_copy = gq7_odom_buf_;
        auto eskf_buf_copy = eskf_odom_buf_;
        // auto log_gq7_task = std::async(std::launch::async, [this]() {
            for (const auto& gq7_odom : gq7_buf_copy) {
                logger_single(2, gq7_odom);
            }
            for (const auto& eskf_odom : eskf_buf_copy) {
                logger_single(3, eskf_odom);
            }
        // });

        // auto log_eskf_task = std::async(std::launch::async, [this]() {

        // });

        // 작업 완료 대기
        // log_gq7_task.get();
        // log_eskf_task.get();
    }

    gq7_odom_buf_.clear();
    eskf_odom_buf_.clear();
}

void SwitchingNode::state_publisher(const nav_msgs::OdometryConstPtr &sub_odom_msg) {
    nav_msgs::Odometry pub_odom_msg = *sub_odom_msg;

    if (!is_stable_switching_) {
        pub_odom_msg = apply_kf(sub_odom_msg);
    }

    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TwistStamped vel_msg;
    odom_pub_.publish(pub_odom_msg);
    pose_msg.header = pub_odom_msg.header;
    pose_msg.header.frame_id = "map";
    pose_msg.pose = pub_odom_msg.pose.pose;
    vel_msg.header = pub_odom_msg.header;
    vel_msg.twist = pub_odom_msg.twist.twist;
    vel_msg.header.frame_id = "map";

    odom_pub_.publish(pub_odom_msg);
    pose_pub_.publish(pose_msg);
    vel_pub_.publish(vel_msg);

    // broadcast tf
    if (pub_tf){
        tf::Transform tf_transform;
        tf_transform.setOrigin(tf::Vector3(
            pub_odom_msg.pose.pose.position.x, 
            pub_odom_msg.pose.pose.position.y, 
            pub_odom_msg.pose.pose.position.z));

        tf::Quaternion tf_quat(
            pub_odom_msg.pose.pose.orientation.x,
            pub_odom_msg.pose.pose.orientation.y,
            pub_odom_msg.pose.pose.orientation.z,
            pub_odom_msg.pose.pose.orientation.w);
        
        tf_transform.setRotation(tf_quat);
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf_transform, pub_odom_msg.header.stamp, frame_id, child_frame_id));
    }

    if (is_stable_switching_){
        prepare_kf(sub_odom_msg);
    }

    num_switching += 1;
}

void SwitchingNode::gq7_callback(const nav_msgs::OdometryConstPtr &gq7_msg) {
    if (!is_initialized_gq7_) is_initialized_gq7_ = true;
    gq7_odom = *gq7_msg;
    gq7_odom_buf_.push_back(gq7_msg);

    if (is_stable_) {
        nav_msgs::OdometryConstPtr odom_ptr;
        // orientation을 이용하여 velocity를 회전시켜 할당
        Eigen::Quaterniond orientation(
            gq7_msg->pose.pose.orientation.w,
            gq7_msg->pose.pose.orientation.x,
            gq7_msg->pose.pose.orientation.y,
            gq7_msg->pose.pose.orientation.z
        );

        Eigen::Vector3d linear_vel(
            gq7_msg->twist.twist.linear.x,
            gq7_msg->twist.twist.linear.y,
            gq7_msg->twist.twist.linear.z
        );

        Eigen::Vector3d transformed_vel = orientation * linear_vel;

        // transformed_vel을 새로운 메시지에 할당
        nav_msgs::OdometryPtr transformed_odom = boost::make_shared<nav_msgs::Odometry>(*gq7_msg);
        transformed_odom->twist.twist.linear.x = transformed_vel.x();
        transformed_odom->twist.twist.linear.y = transformed_vel.y();
        transformed_odom->twist.twist.linear.z = transformed_vel.z();
        state_publisher(transformed_odom);
    }
}

void SwitchingNode::eskf_callback(const nav_msgs::OdometryConstPtr &eskf_msg) {
    if (!is_initialized_eskf_) is_initialized_eskf_ = true;
    eskf_odom = *eskf_msg;
    eskf_odom_buf_.push_back(eskf_msg);

    if (!is_stable_) {
        state_publisher(eskf_msg);
    }
}

}  // namespace ryu

int main(int argc, char **argv) {
    ros::init(argc, argv, "switching");

    ros::NodeHandle nh;
    ryu::SwitchingNode fusion_node(nh);

    ros::spin();

    return 0;
}
