#pragma once
#include "common.h"
#include "frame.h"
#include "map.h"
#include "camera.h"
namespace demo{
    class Map;
    class Backend{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Backend> Ptr;

        Backend();
        void SetCameras(Camera::Ptr camera_left, Camera::Ptr camera_right){
            camera_left_ = camera_left, camera_right_ = camera_right;
        }
        void SetMap(Map::Ptr map){
            map_ = map;

        }

        void UpdateMap();

        void Stop();

    private:
        void BackendLoop();

        /**
         * 对给定的关键帧和路标点进行优化
         * @param keyframes
         * @param landmarks
         */
        void Optimize(Map::KeyFrameType& keyframes, Map::LandmarksType &landmarks);

        std::shared_ptr<Map> map_;
        std::thread backend_thread_;
        std::mutex data_mutex_;

        std::condition_variable map_update_;
        std::atomic<bool> backend_running_;

        Camera::Ptr camera_left_ = nullptr, camera_right_ = nullptr;

    };
}