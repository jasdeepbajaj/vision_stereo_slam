#pragma once
#ifndef VSLAM_DATASET_H
#define VSLAM_DATASET_H

#include "vslam/common_includes.h"
#include "vslam/camera.h"

namespace vslam {
        class Dataset {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            // using Ptr = std::shared_ptr<Dataset>;
            
            Dataset(const std::string& dataset_path);
            bool initialize();
            // Frame::Ptr NextFrame();

            Camera::Ptr GetCamera(int camera_id) const;
        private:
            std::string dataset_path_;
            int current_image_index_{0};

            std::vector<Camera::Ptr> cameras_;
    };
}  // namespace vslam

#endif // VSLAM_DATASET_H