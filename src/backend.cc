#include "backend.h"
#include "utils.h"
#include "feature.h"
#include "mappoint.h"
#include "map.h"

namespace demo{
    Backend::Backend() {
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
    }


    void Backend::BackendLoop() {
        while (backend_running_.load()){
            std::unique_lock<std::mutex> lock(data_mutex_);
            map_update_.wait(lock);
            Map::KeyFrameType  active_kfs = map_->GetActiveKeyFrame();
            Map::LandmarksType active_landmarks = map_->GetAllMapPoint();
            Optimize(active_kfs, active_landmarks);
        }
    }

    void Backend::Optimize(Map::KeyFrameType &keyframes, Map::LandmarksType &landmarks) {
        typedef

    }
}