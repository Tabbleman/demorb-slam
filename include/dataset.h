#pragma once
#include "camera.h"
#include "common.h"
#include "frame.h"
namespace demo{
    /**
     * 配置文件的dataset_dir是数据集路径
     */
     class Dataset{
     public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Dataset> Ptr;
        Dataset(const std::string dataset_path);
        /**
         * 数据集的初始化
         * @return
         */
        bool Init();
        /*
         * 下一帧
         */
        Frame::Ptr NextFrame();
        /*
         * 获取当前的相机
         */
        Camera::Ptr GetCamera(int camera_id) const{
            return camera_.at(camera_id);
        }
     private:
        std::string dataset_path_;
        int current_image_idx = 0;
        std::vector<Camera::Ptr> camera_;
     };
}