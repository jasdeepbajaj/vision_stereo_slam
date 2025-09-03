#pragma once
#ifndef VSLAM_FRAME_H
#define VSLAM_FRAME_H

#include "vslam/common_includes.h"
#include "vslam/feature.h"

namespace vslam {
    class Feature;
    class Frame {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            using Ptr = std::shared_ptr<Frame>;

            // id of the frame
            unsigned long id_ = 0;
            // id of the keyframe
            unsigned long keyframe_id_ = 0;
            // timestamp of the frame
            double time_stamp_ = 0;
            // whether the frame is a keyframe based on multiple criteria
            bool is_keyframe_ = false;
            // pose of the frame : T_cam_world which represents the pose of the camera in the world coordinates (map)
            Sophus::SE3d pose_;
            // left image of the frame
            cv::Mat left_img_;
            // right image of the frame
            cv::Mat right_img_;

            /* Ye mutex pose_ variable ko protect karta hai jab alag-alag threads (jaise frontend aur backend) ek hi time par pose_ ko access ya update karte hain.
            Agar mutex na ho, toh data race ho sakta hai, matlab ek thread pose_ ko update kar raha hai aur dusra thread usi waqt read kar raha hai,
            jis se galat ya inconsistent data mil sakta hai. Isliye, jab bhi pose_ ko read ya write karna ho, pehle mutex lock karo taki
            sabhi threads safely aur sahi tarike se pose_ ka use kar sakein.*/
            std::mutex pose_mutex_;

            // features detected in the left image
            std::vector<Feature::Ptr> features_left_;
            // features detected in the right image -> set to nullptr if corresponding not found
            std::vector<Feature::Ptr> features_right_;

            Frame();
            Frame(long id, double time_stamp, const Sophus::SE3d &pose, const cv::Mat &left, const cv::Mat &right);
            Sophus::SE3d Pose();
            void SetPose(const Sophus::SE3d &pose);
            void SetKeyFrame();
            static Ptr CreateFrame();
    };
}


#endif

