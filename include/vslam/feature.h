#pragma once
#ifndef VSLAM_FEATURE_H
#define VSLAM_FEATURE_H

#include "vslam/common_includes.h"

namespace vslam {
    
    /*Circular dependency ka matlab hota hai jab do ya zyada classes ek dusre ko include karne ki koshish karti hain,
    toh compiler confuse ho jata hai ki pehle kis class ko define kare. Is problem ko avoid karne ke liye hum "forward 
    declaration" use karte hain, jisme sirf class ka naam batate hain bina uski puri definition diye. Isse compiler 
    ko pata chal jata hai ki aisi koi class exist karti hai, aur jab actual definition ki zarurat hoti hai tab hum 
    usko include kar lete hain. Yahan hum Frame aur MapPoint ka forward declaration kar rahe hain taki circular 
    dependency avoid ho sake.*/
    class Frame;
    class MapPoint;

    /**
     * @class Feature
     * @brief Represents a feature point in a stereo vision system.
     */
    class Feature {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            using Ptr = std::shared_ptr<Feature>;

            // Weak pointers to avoid circular dependencies
            std::weak_ptr<Frame> frame_;
            // Keypoint in the image
            cv::KeyPoint keypoint_;
            // Weak pointer to the associated MapPoint
            std::weak_ptr<MapPoint> map_point_;

            // Flag to indicate if the feature is an outlier
            bool is_outlier = false;
            // Flag to indicate if the feature is on the left image
            bool is_on_left_image = true;


            /**
             * @brief Default constructor for the Feature class.
             */
            Feature(){}

            /**
             * @brief Constructor to initialize a Feature with its associated Frame and KeyPoint.
             * @param frame Shared pointer to the frame in which the feature is detected.
             * @param keypoint Keypoint in the image corresponding to this feature.
             */
            Feature(const std::shared_ptr<Frame>& frame, const cv::KeyPoint& keypoint);
    };
}

#endif // VSLAM_FEATURE_H