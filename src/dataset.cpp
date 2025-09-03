#include "vslam/dataset.h"
#include "vslam/vSLAMexception.h"

#include <fstream>

namespace vslam {

    Dataset::Dataset(const std::string& dataset_path) : dataset_path_(dataset_path) {}


    bool Dataset::initialize(){
        std::ifstream fin(dataset_path_ + "/calib.txt");
        
        if (!fin) {
            throw SLAMException("Cannot open kitti calibration parameters file (calib.txt)");
            return false;
        }

        for (int i = 0; i < 4; i++) {
            
            char camera_name[3];
            for (int k = 0; k < 3; ++k) fin >> camera_name[k];

            double projection_data[12];
            for (int k = 0; k < 12; ++k) fin >> projection_data[k];
            
            /* Since images are rectified in the KITTI dataset,
            * the projection matrix general form from K[R|t] simplifies
            * to K[I|t] or [K|Kt]. */
            Eigen::Matrix<double, 3, 3>  K;
            K << projection_data[0], projection_data[1], projection_data[2],
                projection_data[4], projection_data[5], projection_data[6],
                projection_data[8], projection_data[9], projection_data[10];
            
            Eigen::Matrix<double, 3, 1>  t;
            t << projection_data[3], projection_data[7], projection_data[11];
            t = K.inverse() * t;

            double baseline = t.norm();
            Sophus::SO3d R; //R = I by default since kitti data is rectified already
            Sophus::SE3d camera_pose(R, t); //T_cam_refCam (Here we use reference camera as left camera which is grayscale camera 0)

            //adjust K since we use down sampled kitti images
            K *= 0.5;

            Camera::Ptr new_camera;
            new_camera = std::make_shared<Camera>(K(0, 0), K(1, 1), K(0, 2), K(1, 2), baseline, camera_pose);
            cameras_.push_back(new_camera);
            // LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
        }
        fin.close();
        current_image_index_ = 0;

        return true;
    }

    // Frame::Ptr NextFrame() {
    // }

    Camera::Ptr Dataset::GetCamera(int camera_id) const {
        return cameras_.at(camera_id);
    }

}  // namespace vslam