#include "vslam/camera.h"

namespace vslam {

    Camera::Camera(double fx, double fy, double cx, double cy, double baseline, const Sophus::SE3d &pose) : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose){
         pose_inv_ = pose_.inverse();
    }

    Sophus::SE3d Camera::pose() const {
        return pose_;
    }

    Eigen::Matrix3d Camera::K() const {
        Eigen::Matrix3d K;
        K << fx_,  0 , cx_, 
              0 , fy_, cy_, 
              0 ,  0 ,  1;
        return K;
    }

    Eigen::Vector3d Camera::world2camera(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w){
        return pose_ * T_c_w * p_w;
    }

    Eigen::Vector3d Camera::camera2world(const Eigen::Vector3d &p_c, const Sophus::SE3d &T_c_w ) {
        return T_c_w.inverse() * pose_inv_ * p_c;
    }

    Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d &p_c) {
        double u = fx_ * p_c(0, 0) / p_c(2, 0) + cx_;
        double v = fy_ * p_c(1, 0) / p_c(2, 0) + cy_;
        return Eigen::Vector2d(u, v);
    }

    Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d &p_p, double depth) {
        double z = depth;
        double x = (p_p(0, 0) - cx_) * z / fx_;
        double y = (p_p(1, 0) - cy_) * z / fy_;
        return Eigen::Vector3d(x, y, z);
    }

    Eigen::Vector2d Camera::world2pixel(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w) {
        return camera2pixel(world2camera(p_w, T_c_w));
    }

    Eigen::Vector3d Camera::pixel2world(const Eigen::Vector2d &p_p, const Sophus::SE3d &T_c_w, double depth) {
        return camera2world(pixel2camera(p_p, depth), T_c_w);
    }

    // std::ostream& operator<<(std::ostream &os, const Camera &obj){   
    //     os << "fx: " << obj.fx_ <<
    //           ", fy: " << obj.fy_ <<
    //           ", cx: " << obj.cx_ <<
    //           ", cy: " << obj.cy_ <<
    //           ", baseline: " << obj.baseline_ <<
    //           "\npose-r:\n" << obj.pose_.rotationMatrix() <<
    //           "\npose-t:\n" << obj.pose_.translation();
    //     return os;
    // }
}