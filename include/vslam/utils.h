#ifndef VSLAM_UTILS_H
#define VSLAM_UTILS_H

#include "vslam/common_includes.h"

namespace vslam {
    /**
     * @brief This functions takes n corresponding points in n camera poses in stereo coordinates system, then calculates the 3D position of the points
     * in the stereo coordinate system.
     * @param poses std::vector<Sophus::SE3d> The camera poses in the stereo coordinate system.
     * @param points std::vector<Eigen::Vector3d> Corresponding points in the left and right camera normal planes.
     * @param p_w Eigen::Vector3d& The 3D point in the stereo coordinate system (output).
     * @return true if triangulation is successful, false otherwise.
     * 
     *          The function uses the Direct Linear Transformation (DLT) algorithm to perform triangulation.
     *          Assume there are two cameras:
     *
     *          point1 : (x1, y1, 1)
     *          point2 : (x2, y2, 1)
     *
     *          points = {point1, point2}
     *
     *          The corresponding 3D point in the stereo coordinate system can be obtained by triangulating
     *          the two 2D points using the camera poses.
     *          
     *          pose1 or P1 = [r1; r2; r3], pose1 is 3X4 where ri is a row of pose1
     *          pose2 or P2 = [q1; q2; q3], pose2 is 3X4 where qi is a row of pose2
     *
     *          poses = {pose1, pose2}
     * 
     *          The DLT algorithm is as follows:
     *
     *          p_w is the corresponding 3D point to the 2D points in the stereo coordinate system.
     *          P1 * pw = p1  (project point in world into camera 1)
     *          P2 * pw = p2  (project point in world into camera 2)
     * 
     *          P1 * pw = p1 =>             (1) r1 * w = x1
     *                                      (2) r2 * w = y1
     *                                      (3) r3 * w =  1
     * 
     *          P2 * pw = p2 =>             (4) q1 * w = x2
     *                                      (5) q2 * w = y2
     *                                      (6) q3 * w =  1
     *      
     *          (1),(3): (x1 * r3 - r1) * w = 0
     *          (2),(3): (y1 * r3 - r2) * w = 0   
     *          (4),(5): (x2 * q3 - q1) * w = 0
     *          (5),(6): (y2 * q3 - q2) * w = 0
     *
     *          The final system of equations can be represented in matrix form as:
     *
     *          A * w = 0
     *
     *          where A is a 4x4 matrix and w is the 3D point which can be obtained by Singular Value Decomposition (SVD).
     */
    inline bool triangulation(const std::vector<Sophus::SE3d>& poses, const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& p_w) {

        Eigen::Matrix<double, Eigen::Dynamic, 4> A(2 * poses.size(), 4);
        Eigen::Matrix<double, Eigen::Dynamic, 1> b(2 * poses.size());

        b.setZero();
        for (size_t i = 0; i < poses.size(); i++){
            Eigen::Matrix<double, 3, 4> P = poses[i].matrix3x4();
            Eigen::Vector3d p = points[i];

            A.row(2 * i)     = p(0, 0) * P.row(2) - P.row(0); // (x * r3 - r1)
            A.row(2 * i + 1) = p(1, 0) * P.row(2) - P.row(1); // (y * r3 - r2)
        }

        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Vector4d homogeneous_point = svd.matrixV().col(3);
        p_w = (homogeneous_point / homogeneous_point[3]).head<3>();

        double smallest = svd.singularValues()[3];
        double second_smallest = svd.singularValues()[2];

        if (smallest / second_smallest < 1e-2) {
            return true;  
        }

        return false;  
    }

    /**
     * @brief Convert OpenCV 2D point to Eigen 2D vector
     * @param p cv::Point2f OpenCV 2D point
     */
    inline Eigen::Vector2d toVec2(const cv::Point2f p) { return Eigen::Vector2d(p.x, p.y); }
}

#endif