#pragma once
#ifndef VSLAM_MAP_POINT_H
#define VSLAM_MAP_POINT_H

#include "vslam/common_includes.h"
#include "vslam/feature.h"

namespace vslam {

    class Frame;
    class Feature;

    class MapPoint {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            using Ptr = std::shared_ptr<MapPoint>;

            // data members
            unsigned long id_ = 0;
            bool is_outlier_ = false;
            Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();
            std::mutex data_mutex_;
            int observed_times_ = 0; 
            std::list<std::weak_ptr<Feature>> observations_;

            // constructors
            MapPoint() {}
            MapPoint(unsigned long id, const Eigen::Vector3d& position);

            // getters and setters
            Eigen::Vector3d Pos();
            void SetPos(const Eigen::Vector3d &pos);
            void AddObservation(Feature::Ptr feature);
            void RemoveObservation(Feature::Ptr feature);
            std::list<std::weak_ptr<Feature>> GetObs();

            // factory function -> assigning ids
            static MapPoint::Ptr CreateNewMappoint();

    };

} // namespace vslam

#endif  // VSLAM_MAP_POINT_H