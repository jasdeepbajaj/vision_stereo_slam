#include "vslam/map_point.h"

namespace vslam {

    MapPoint::MapPoint(unsigned long id, const Eigen::Vector3d& position) : id_(id), pos_(position) {}
    
    Eigen::Vector3d MapPoint::Pos() {
        std::unique_lock<std::mutex> lk(data_mutex_);
        return pos_;
    }

    void MapPoint::SetPos(const Eigen::Vector3d &pos) {
        std::unique_lock<std::mutex> lk(data_mutex_);
        pos_ = pos;
    }

    void MapPoint::AddObservation(Feature::Ptr feature) {
        std::unique_lock<std::mutex> lk(data_mutex_);
        observations_.push_back(feature);
        observed_times_++;
    }

    void MapPoint::RemoveObservation(Feature::Ptr feature) {
        std::unique_lock<std::mutex> lk(data_mutex_);
        for (auto iter = observations_.begin(); iter != observations_.end(); iter++) {
            if (iter->lock() == feature) {
                observations_.erase(iter);
                observed_times_--;
                return;
            }
        }
    }

    std::list<std::weak_ptr<Feature>> MapPoint::GetObs() {
        std::unique_lock<std::mutex> lk(data_mutex_);
        return observations_;
    }

    MapPoint::Ptr MapPoint::CreateNewMappoint() {
        static unsigned long factory_id = 0;
        Ptr new_map_point = std::make_shared<MapPoint>();
        new_map_point->id_ = factory_id++;
        return new_map_point;
    }
}