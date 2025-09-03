#include "vslam/map.h"

namespace vslam{
    Map::Map(){}

    void Map::insertKeyFrame(Frame::Ptr frame) {
        current_frame_ = frame;

        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;

        if (active_keyframes_.size() > static_cast<size_t>(num_active_keyframes_)) removeOldKeyFrames();
    }

    void Map::insertMapPoint(MapPoint::Ptr map_point) {
        landmarks_[map_point->id_] = map_point;
        active_landmarks_[map_point->id_] = map_point;
    }

    void Map::removeOldKeyFrames() {
        if (current_frame_ == nullptr) return;
        
        double max_dis = 0, min_dis = 9999;
        double max_kf_id = 0, min_kf_id = 0;

        Sophus::SE3d T_world_curr = current_frame_->Pose().inverse();

        for(const auto& [id, kf] : active_keyframes_) {
            if (kf == current_frame_) continue;
            Sophus::SE3d T_kf_world = kf->Pose();
            Sophus::SE3d T_kf_camera = T_kf_world * T_world_curr; // T_kf_camera = T_kf_world * T_world_camera
            double dis = T_kf_camera.log().norm(); //distance between keyframe and current frame

            if (dis > max_dis) {max_dis = dis; max_kf_id = id;} // farthest keyframe
            if (dis < min_dis) {min_dis = dis; min_kf_id = id;} // nearest keyframe
        }


        const double min_dis_th = 0.2; // threshold for minimum distance
        Frame::Ptr frame_to_remove = nullptr;

        if (min_dis < min_dis_th) frame_to_remove = keyframes_.at(min_kf_id); // If there are very recent frames, delete the most recent one first.
        else frame_to_remove = keyframes_.at(max_kf_id); // If all frames are older, delete the farthest one.

        // LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;

        // remove keyframe from active keyframes (hash_map<int, Frame::Ptr>)
        active_keyframes_.erase(frame_to_remove->keyframe_id_);

        // remove links between map points(landmarks) and keypoints featuresof removed key frame
        for (auto feat_ptr : frame_to_remove->features_left_) {
            if (feat_ptr == nullptr) continue;
            MapPoint::Ptr mp = feat_ptr->map_point_.lock();
            if (mp) mp->RemoveObservation(feat_ptr); 
        }

        // remove links between map points(landmarks) and keypoints features of removed key frame
        for (auto feat_ptr : frame_to_remove->features_right_) {
            if (feat_ptr == nullptr) continue;
            auto mp = feat_ptr->map_point_.lock();
            if (mp) mp->RemoveObservation(feat_ptr);
        }

        cleanUp();
    }

    void Map::cleanUp() {
        int cnt_landmark_removed = 0;
        for (auto iter = active_landmarks_.begin(); iter != active_landmarks_.end();) {
            if (iter->second->observed_times_ == 0) { iter = active_landmarks_.erase(iter); cnt_landmark_removed++;} 
            else ++iter;
        }
        // LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";
    }

    Map::LandmarksType Map::GetAllMapPoints() {
        std::unique_lock<std::mutex> lk(data_mutex_);
        return landmarks_;
    }
    
    
    Map::KeyframesType Map::GetAllKeyFrames() {
        std::unique_lock<std::mutex> lk(data_mutex_);
        return keyframes_;
    }


    Map::LandmarksType Map::GetActiveMapPoints(){
        std::unique_lock<std::mutex> lk(data_mutex_);
        return active_landmarks_;
    }


    Map::KeyframesType Map::GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lk(data_mutex_);
        return active_keyframes_;
    }

}