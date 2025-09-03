#pragma once
#ifndef VSLAM_CAMERA_H
#define VSLAM_CAMERA_H

#include "vslam/common_includes.h"

namespace vslam{

    /**
     * @class Camera
     * @brief Represents a pinhole stereo camera model with intrinsic and extrinsic parameters.
     * 
     * Provides methods for coordinate transformations between world, camera, and pixel frames.
     */
    class Camera {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            using Ptr = std::shared_ptr<Camera>;
            /// Camera intrinsic parameters: focal lengths and principal point.
            double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};

            /// Stereo baseline (distance between the two cameras).
            double baseline_{0.0};

            // pose_ represents the pose of the camera in stereosystem
            // example:
            // if left camera is the reference then for the right camera
            // this represents the position of the right camera w.r.t. left camera
            // T_cam_refCam
            Sophus::SE3d pose_;

            //T_refCam_cam
            Sophus::SE3d pose_inv_;

            /**
             * @brief Default constructor.
             */
            // Camera();

            /**
             * @brief Parameterized constructor.
             * @param fx Focal length in x.
             * @param fy Focal length in y.
             * @param cx Principal point x.
             * @param cy Principal point y.
             * @param baseline Stereo baseline.
             * @param pose pose represents the pose of the camera in stereosystem
             */
            Camera(double fx, double fy, double cx, double cy, double baseline, const Sophus::SE3d &pose);

            /**
             * @brief Get the current pose of the camera.
             * @return Sophus::SE3d Pose of the camera in stereosystem.
             */
            Sophus::SE3d pose() const;

            /**
             * @brief Get the camera intrinsic matrix.
             * @return Eigen::Matrix3d 3x3 camera intrinsic matrix K.
             */
            Eigen::Matrix3d K() const;

            /**
             * @brief Transform a 3D point from world coordinates (map) to camera coordinate.
             * @param p_w Point in world coordinates.
             * @param T_c_w represents the pose of the stero system in the world coordinate system
             * @return Eigen::Vector3d Point in camera coordinates.
             */
            Eigen::Vector3d world2camera(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w);

            /**
             * @brief Transform a 3D point from camera coordinate to world coordinate.
             * @param p_c Point in camera coordinates.
             * @param T_c_w represents the pose of the stero system in the world coordinate system
             * @return Eigen::Vector3d Point in world coordinates.
             */
            Eigen::Vector3d camera2world(const Eigen::Vector3d &p_c, const Sophus::SE3d &T_c_w);

            /**
             * @brief Project a 3D point from camera coordinates to pixel coordinates.
             * @param p_c Point in camera coordinates.
             * @return Eigen::Vector2d Point in pixel coordinates.
             */
            Eigen::Vector2d camera2pixel(const Eigen::Vector3d &p_c);

            /**
             * @brief Find the corresponding 3D point of a pixel in the camera coordinate system.
             * If depth is equal to 1, it finds the corresponding point of the pixel in the camera's normal plane. 
             * p_camera_normal_plane = inverse(K) * p_p
             * p_c = p_camera_normal_plane * depth
             * @param p_p Point in pixel coordinates.
             * @param depth Depth value at the pixel.
             * @return Eigen::Vector3d Point in camera coordinates.
             */
            Eigen::Vector3d pixel2camera(const Eigen::Vector2d &p_p, double depth = 1);

            /**
             * @brief Transform a 3D world point directly to pixel coordinates.
             * @param p_w Point in world coordinates.
             * @param T_c_w Represents the pose of the stero system in the world coordinate system
             * @return Eigen::Vector2d Point in pixel coordinates.
             */
            Eigen::Vector2d world2pixel(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w);

            /**
             * @brief Back-project a pixel to world coordinates using depth and camera pose.
             * @param p_p Pixel coordinates.
             * @param T_c_w Represents the pose of the stero system in the world coordinate system
             * @param depth Depth value.
             * @return Eigen::Vector3d Point in world coordinates.
             */
            Eigen::Vector3d pixel2world(const Eigen::Vector2d &p_p, const Sophus::SE3d &T_c_w, double depth);

    };

} // namespace vslam

#endif  // VSLAM_CAMERA_H