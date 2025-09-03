#include "vslam/feature.h"

namespace vslam {
    Feature::Feature(const std::shared_ptr<Frame>& frame, const cv::KeyPoint& keypoint) : frame_(frame), keypoint_(keypoint) {}
}