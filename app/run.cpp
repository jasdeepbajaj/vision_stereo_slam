#include "vslam/common_includes.h"
#include "vslam/camera.h"
#include "vslam/dataset.h"


int main()
{
    std::string dataset_path_str{"../stereo_slam/dataset/sequences/00"};
    vslam::Dataset ds(dataset_path_str);

    ds.initialize();

    return 0;
}