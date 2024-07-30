#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <deque>
#include <fstream>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include "microstrain_inertial_msgs/HumanReadableStatus.h"
#include <tf/transform_broadcaster.h>


namespace ryu {

class SwitchingNode {
   public:
    SwitchingNode(ros::NodeHandle &nh) {
        std::string topic_status, topic_gq7, topic_eskf, topic_pubodom, topic_pubpose, topic_pubvel, log_path; 
        nh.param<std::string>("topic_status", topic_status, "/ekf/status");
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
        gq7_sub_ = nh.subscribe(topic_gq7, 10, &SwitchingNode::gq7_callback, this);
        eskf_sub_ = nh.subscribe(topic_eskf, 10, &SwitchingNode::eskf_callback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>(topic_pubodom, 10);

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
        nh.param("num_stable_threshold", num_stable_threshold, 5);

        stable_count=0;
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
    void state_publisher(const nav_msgs::OdometryConstPtr &odom_msg);
    void gq7_callback(const nav_msgs::OdometryConstPtr &gq7_msg);
    void eskf_callback(const nav_msgs::OdometryConstPtr &eskf_msg);
    void logger(bool stable_flag);

   private:
    ros::Subscriber status_sub_;
    ros::Subscriber gq7_sub_;
    ros::Subscriber eskf_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher vel_pub_;
    tf::TransformBroadcaster tf_broadcaster_;


    bool is_initialized_gq7_ = false;
    bool is_initialized_eskf_ = false;
    bool is_stable_ = false;

    nav_msgs::Odometry gq7_odom;
    nav_msgs::Odometry eskf_odom;
    std::ofstream log_file_;
    bool pub_tf;
    std::string frame_id, child_frame_id;
    int stable_count, num_stable_threshold;
};

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

void SwitchingNode::status_callback(const microstrain_inertial_msgs::HumanReadableStatusConstPtr &status_msg) {
    int num_status = status_msg->status_flags.size();
    std::string first_status = status_msg->status_flags[0];
    std::string stable_status = "\"Stable\""; // 따옴표를 그대로 사용

    if (num_status == 1 && first_status == stable_status) {
        if (!is_stable_){
            stable_count++;    
        }
        if (stable_count >= num_stable_threshold) {
            if (!is_stable_){
                is_stable_ = true;
            }
        }
        logger(is_stable_);

    } else {
        stable_count = 0;
        if (is_stable_){
            is_stable_ = false;
        }
        logger(is_stable_);
        // std::cout << "unstable" << std::endl;
    }
}
void SwitchingNode::state_publisher(const nav_msgs::OdometryConstPtr &sub_odom_msg) {
    nav_msgs::Odometry pub_odom_msg = *sub_odom_msg;

    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TwistStamped  vel_msg;
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
}

void SwitchingNode::gq7_callback(const nav_msgs::OdometryConstPtr &gq7_msg) {
    if (!is_initialized_gq7_) is_initialized_gq7_ = true;
    gq7_odom = *gq7_msg;
    if (is_stable_) {
        state_publisher(gq7_msg);
    }
}

void SwitchingNode::eskf_callback(const nav_msgs::OdometryConstPtr &eskf_msg) {
    if (!is_initialized_eskf_) is_initialized_eskf_ = true;
    eskf_odom = *eskf_msg;
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
