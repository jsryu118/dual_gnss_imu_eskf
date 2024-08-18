#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <deque>

#include "utils.h"

namespace ryu {

// constexpr int kStateDim = 15;
// constexpr int kNoiseDim = 12;

//TODORYU
// Ulsan UNIST 3dmgq7
// constexpr double kG = 9.78662;
// Daegu PG 3dmgq7
// constexpr double kG = 9.80186;

// using MatrixSD = Eigen::Matrix<double, kStateDim, kStateDim>;

class Gnss {
   public:
        bool is_initialized;
        Eigen::Vector3d enu_front;
        Eigen::Vector3d enu_behind;
        Eigen::Vector3d lla_front;
        Eigen::Vector3d lla_behind;

        Eigen::Vector3d gnss_main_to_sub_global;
        Eigen::Vector3d intial_position_sub;
        Eigen::Vector3d intial_position_main;


    Gnss() = delete;

    Gnss(const Gnss &) = delete;

    explicit Gnss(Eigen::Vector3d origin_enu, bool is_dual, int kGnssBufSize, double gnss_std_threshold,
        double estimated_distance, double delta_gnss_threshold)
        : origin_enu_(origin_enu), is_dual_(is_dual), GnssBufSize_(kGnssBufSize), 
        gnss_std_threshold_(gnss_std_threshold), estimated_distance_(estimated_distance),
        delta_gnss_threshold_(delta_gnss_threshold) {
        is_initialized = false;
        enu_front.setZero();
        enu_behind.setZero();
        lla_front.setZero();
        lla_behind.setZero();
        intial_position_sub.setZero();
        intial_position_main.setZero();
        gnss_sub_buf_.clear();
        gnss_main_buf_.clear();
        gnss_main_to_sub_global = Eigen::Vector3d(1., 0., 0.);
    }

    void initializer(gnssDataPtr gnss_ptr, bool is_sub) {
        if(is_sub){
            gnss_sub_buf_.push_back(gnss_ptr);
            if (gnss_sub_buf_.size() > GnssBufSize_) gnss_sub_buf_.pop_front();
        }
        else{
            gnss_main_buf_.push_back(gnss_ptr);
            if (gnss_main_buf_.size() > GnssBufSize_) gnss_main_buf_.pop_front();
        }
    }

    // bool check_dual_heading_fix(){
    //     return true;
    // }

    // void initializeSingelGnss() {
    //     if(!(gnss_main_buf_.size() == GnssBufSize_)) return;
    //     Eigen::Vector3d sum_main(0., 0., 0.);

    //     std::vector<Eigen::Vector3d> gnss_main_enu_vec;

    //     for (const auto gnss_main : gnss_main_buf_) {
    //         Eigen::Vector3d gnss_main_enu; // position of gnss from global(map) frame
    //         ryu::lla2enu(origin_enu_, gnss_main->lla, &gnss_main_enu);
    //         sum_main += gnss_main_enu;
    //         gnss_main_enu_vec.push_back(gnss_main_enu);
    //     }
        
    //     intial_position_main = sum_main / (double)gnss_main_buf_.size();

    //     Eigen::Vector3d sum_err2_sub(0., 0., 0.);
    //     Eigen::Vector3d sum_err2_main(0., 0., 0.);
    //     for (const auto gnss_sub_enu : gnss_sub_enu_vec) sum_err2_sub += (gnss_sub_enu - intial_position_sub).cwiseAbs2();
    //     for (const auto gnss_main_enu : gnss_main_enu_vec) sum_err2_main += (gnss_main_enu - intial_position_main).cwiseAbs2();
    //     const Eigen::Vector3d std_gnss_sub = (sum_err2_sub / (double)gnss_sub_buf_.size()).cwiseSqrt();
    //     const Eigen::Vector3d std_gnss_main = (sum_err2_main / (double)gnss_main_buf_.size()).cwiseSqrt();

    //     bool std_flag_sub_ = true;
    //     bool std_flag_main_ = true;
    //     if (std_gnss_sub.maxCoeff() > gnss_std_threshold_) {
    //         printf("[gnss_imu %s] Too big gnss_sub_std for heading initialization: (%f, %f, %f)!!!\n", __FUNCTION__, std_gnss_sub[0], std_gnss_sub[1], std_gnss_sub[2]);
    //         gnss_sub_buf_.clear();
    //         std_flag_sub_ = false;
    //     }
    //     if (std_gnss_main.maxCoeff() > gnss_std_threshold_) {
    //         printf("[gnss_imu %s] Too big gnss_main_std for heading initialization: (%f, %f, %f)!!!\n", __FUNCTION__, std_gnss_main[0], std_gnss_main[1], std_gnss_main[2]);
    //         gnss_main_buf_.clear();
    //         std_flag_main_ = false;
    //     }
    //     if (!(std_flag_sub_ && std_flag_main_)) return;

    //     double measured_distance = (intial_position_sub - intial_position_main).norm();
    //     double distance_err = std::abs(measured_distance - estimated_distance_);

    //     if (distance_err > delta_gnss_threshold_){
    //         printf("[gnss_imu %s] two gnss are not located within estimated distance! : %f m !!!\n", __FUNCTION__, distance_err);
    //         gnss_sub_buf_.clear();
    //         gnss_main_buf_.clear();
    //         return;
    //     }
    //     gnss_main_to_sub_global = intial_position_sub - intial_position_main;
    //     is_initialized = true;
    // } 

    bool checkGnssBuf(std::deque<gnssDataConstPtr>& gnss_buf_, Eigen::Vector3d& intial_position) {
        if(!(gnss_buf_.size() == GnssBufSize_)) return false;
        
        Eigen::Vector3d sum(0., 0., 0.);

        std::vector<Eigen::Vector3d> gnss_enu_vec;

        for (const auto gnss_ : gnss_buf_) {
            Eigen::Vector3d gnss_enu; // position of gnss from global(map) frame
            ryu::lla2enu(origin_enu_, gnss_->lla, &gnss_enu);
            sum += gnss_enu;
            gnss_enu_vec.push_back(gnss_enu);
        }
        
        intial_position = sum / (double)gnss_buf_.size();        
        Eigen::Vector3d sum_err2(0., 0., 0.);
        for (const auto gnss_enu : gnss_enu_vec) sum_err2 += (gnss_enu - intial_position).cwiseAbs2();
        const Eigen::Vector3d std_gnss = (sum_err2 / (double)gnss_buf_.size()).cwiseSqrt();

        if (sum_err2.maxCoeff() > gnss_std_threshold_) {
            printf("[gnss_imu %s] Too big gnss_sub_std for heading initialization: (%f, %f, %f)!!!\n", __FUNCTION__, sum_err2[0], sum_err2[1], sum_err2[2]);
            gnss_buf_.clear();
            return false;
        }
        else{
            return true;
        }
    }
    void computeHeadingDualGnss() {

        bool main_check = checkGnssBuf(gnss_main_buf_, intial_position_main);
        bool sub_check = checkGnssBuf(gnss_sub_buf_, intial_position_sub);
        if (!(main_check && sub_check)) return; 

        double measured_distance = (intial_position_sub - intial_position_main).norm();
        double distance_err = std::abs(measured_distance - estimated_distance_);

        if (distance_err > delta_gnss_threshold_){
            printf("[gnss_imu %s] two gnss are not located within estimated distance! : %f m !!!\n", __FUNCTION__, distance_err);
            gnss_sub_buf_.clear();
            gnss_main_buf_.clear();
            return;
        }
        gnss_main_to_sub_global = intial_position_sub - intial_position_main;
        is_initialized = true;
    }

    void initializeSingleGnss() {
        if(checkGnssBuf(gnss_main_buf_, intial_position_main)) is_initialized = true;
    }
    // void computeHeadingDualGnss() {
    //     if(!(gnss_sub_buf_.size() == GnssBufSize_ && gnss_main_buf_.size() == GnssBufSize_)) return;
    //     Eigen::Vector3d sum_sub(0., 0., 0.);
    //     Eigen::Vector3d sum_main(0., 0., 0.);

    //     std::vector<Eigen::Vector3d> gnss_sub_enu_vec;
    //     std::vector<Eigen::Vector3d> gnss_main_enu_vec;

    //     for (const auto gnss_sub : gnss_sub_buf_) {
    //         Eigen::Vector3d gnss_sub_enu; // position of gnss from global(map) frame
    //         ryu::lla2enu(origin_enu_, gnss_sub->lla, &gnss_sub_enu);
    //         sum_sub += gnss_sub_enu;
    //         gnss_sub_enu_vec.push_back(gnss_sub_enu);
    //     }
    //     for (const auto gnss_main : gnss_main_buf_) {
    //         Eigen::Vector3d gnss_main_enu; // position of gnss from global(map) frame
    //         ryu::lla2enu(origin_enu_, gnss_main->lla, &gnss_main_enu);
    //         sum_main += gnss_main_enu;
    //         gnss_main_enu_vec.push_back(gnss_main_enu);
    //     }
        
    //     intial_position_sub = sum_sub / (double)gnss_sub_buf_.size();        
    //     intial_position_main = sum_main / (double)gnss_main_buf_.size();

    //     Eigen::Vector3d sum_err2_sub(0., 0., 0.);
    //     Eigen::Vector3d sum_err2_main(0., 0., 0.);
    //     for (const auto gnss_sub_enu : gnss_sub_enu_vec) sum_err2_sub += (gnss_sub_enu - intial_position_sub).cwiseAbs2();
    //     for (const auto gnss_main_enu : gnss_main_enu_vec) sum_err2_main += (gnss_main_enu - intial_position_main).cwiseAbs2();
    //     const Eigen::Vector3d std_gnss_sub = (sum_err2_sub / (double)gnss_sub_buf_.size()).cwiseSqrt();
    //     const Eigen::Vector3d std_gnss_main = (sum_err2_main / (double)gnss_main_buf_.size()).cwiseSqrt();

    //     bool std_flag_sub_ = true;
    //     bool std_flag_main_ = true;
    //     if (std_gnss_sub.maxCoeff() > gnss_std_threshold_) {
    //         printf("[gnss_imu %s] Too big gnss_sub_std for heading initialization: (%f, %f, %f)!!!\n", __FUNCTION__, std_gnss_sub[0], std_gnss_sub[1], std_gnss_sub[2]);
    //         gnss_sub_buf_.clear();
    //         std_flag_sub_ = false;
    //     }
    //     if (std_gnss_main.maxCoeff() > gnss_std_threshold_) {
    //         printf("[gnss_imu %s] Too big gnss_main_std for heading initialization: (%f, %f, %f)!!!\n", __FUNCTION__, std_gnss_main[0], std_gnss_main[1], std_gnss_main[2]);
    //         gnss_main_buf_.clear();
    //         std_flag_main_ = false;
    //     }
    //     if (!(std_flag_sub_ && std_flag_main_)) return;

    //     double measured_distance = (intial_position_sub - intial_position_main).norm();
    //     double distance_err = std::abs(measured_distance - estimated_distance_);

    //     if (distance_err > delta_gnss_threshold_){
    //         printf("[gnss_imu %s] two gnss are not located within estimated distance! : %f m !!!\n", __FUNCTION__, distance_err);
    //         gnss_sub_buf_.clear();
    //         gnss_main_buf_.clear();
    //         return;
    //     }
    //     gnss_main_to_sub_global = intial_position_sub - intial_position_main;
    //     is_initialized = true;
    // }

    ~Gnss() {}

    private:
        bool is_dual_;
        Eigen::Vector3d origin_enu_;
        int GnssBufSize_;
        std::deque<gnssDataConstPtr> gnss_sub_buf_;
        std::deque<gnssDataConstPtr> gnss_main_buf_;
        double gnss_std_threshold_;
        double estimated_distance_;
        double delta_gnss_threshold_;

};



using GnssPtr = std::unique_ptr<Gnss>;

}  // namespace ryu