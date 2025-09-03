#ifndef VSLAM_MAP_H
#define VSLAM_MAP_H

#include "vslam/common_includes.h"
#include "vslam/mappoint.h"
#include "vslam/frame.h"

namespace vslam {
    class Map {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            using Ptr = std::shared_ptr<Map>;
            using LandmarksType = std::unordered_map<unsigned long, MapPoint::Ptr>;
            using KeyframesType = std::unordered_map<unsigned long, Frame::Ptr>;

            Map();

            void cleanUp();
            void insertKeyFrame(Frame::Ptr frame);
            void insertMapPoint(MapPoint::Ptr map_point);
            LandmarksType GetAllMapPoints();
            KeyframesType GetAllKeyFrames();
            LandmarksType GetActiveMapPoints();
            KeyframesType GetActiveKeyFrames();

        private:
            std::mutex data_mutex_;
            
            // All landmarks; Hashtable (id-landmark(map point))
            LandmarksType landmarks_;
            // Active landmarks; Hashtable (id-landmark(map point))
            LandmarksType active_landmarks_;
            
            // All key frames; Hashtable (id-keyframe)
            KeyframesType keyframes_;
            // Active key frames; Hashtable (id-keyframe)
            KeyframesType active_keyframes_;

            Frame::Ptr current_frame_ = nullptr;

            // Hyper-parameter: max number of active keyframes
            int num_active_keyframes_ = 9;

            void removeOldKeyFrames();
    };

}

#endif