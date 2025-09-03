#pragma once
#ifndef VSLAM_DATASET_H
#define VSLAM_DATASET_H

#include "vslam/common_includes.h"
#include "vslam/camera.h"
#include "vslam/frame.h"

namespace vslam {

    /**
     * @class Dataset
     * @brief Stereo vision SLAM ke liye dataset ko handle karne wali class.
     *
     * Ye class dataset ka path, cameras, aur images ka index manage karti hai.
     * Iska use karke hum dataset initialize kar sakte hain, nayi frames read kar sakte hain,
     * aur camera parameters access kar sakte hain.
     */
    class Dataset {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            using Ptr = std::shared_ptr<Dataset>;

            /**
             * @brief Constructor - Dataset ka path set karta hai.
             * @param dataset_path Dataset ki root directory ka path (jahan images aur calib.txt hai)
             * Example: "/home/user/KITTI/sequences/00"
             */
            Dataset(const std::string& dataset_path);

            /**
             * @brief Dataset ko initialize karta hai, calibration file se camera parameters read karta hai.
             * Ye function calib.txt file ko padhta hai, har camera ke projection matrix se intrinsic (K) aur translation (t) nikalta hai,
             * aur Camera objects banata hai.
             * @return true agar initialization successful ho, false otherwise.
             */
            bool initialize();

            /**
             * @brief Next stereo frame load karta hai (left aur right image).
             * Ye function current_image_index_ ke hisaab se left aur right images load karta hai,
             * unko resize karta hai (downsample by 2), aur ek naya Frame object return karta hai.
             * Agar images nahi milti toh nullptr return karta hai.
             * @return Frame::Ptr Naya frame object, ya nullptr agar images nahi milti.
             */
            Frame::Ptr NextFrame();

            /**
             * @brief Camera object return karta hai given camera_id ke liye.
             * @param camera_id Camera ka index (0 = left, 1 = right)
             * @return Camera::Ptr Camera object pointer
             */
            Camera::Ptr GetCamera(int camera_id) const;

        private:
            /// @brief Dataset ka root path (jahan images aur calib.txt hai)
            std::string dataset_path_;

            /// @brief Current image index (kaunsi image read karni hai)
            int current_image_index_ = 0;

            /// @brief Left camera ka id (default: 0)
            int left_cam_id_ = 0;

            /// @brief Right camera ka id (default: 1)
            int right_cam_id_ = 1;

            /// @brief OpenCV flag for reading images (default: grayscale)
            int flag_read_img_ = cv::IMREAD_GRAYSCALE;

            /// @brief Cameras ka vector (har camera ke parameters yahan store hote hain)
            std::vector<Camera::Ptr> cameras_;
    };

    }  // namespace vslam

#endif // VSLAM_DATASET_H