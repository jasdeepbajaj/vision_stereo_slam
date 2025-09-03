#ifndef VSLAM_CONFIG_H
#define VSLAM_CONFIG_H

#include "vslam/common_includes.h"

namespace vslam {
    // Class to obtain hyper-parameters and setting from file (Singleton pattern)
    class Config {
        private:
            // Singleton object
            static std::shared_ptr<Config> config_;

            // For reading yaml config file
            cv::FileStorage file_;
            
            Config(){};
            Config(Config &obj);
            Config& operator=(const Config& other);

        public:
            ~Config();

            // Set a new config file
            static bool SetParameterFile(const std::string &filename);

            // Access the parameter values
            template <typename T>
            static T Get(const std::string &key) {
                return T(Config::config_->file_[key]); 
            }
    };   

}

#endif