#ifndef DEMO_MAP_H
#define DEMO_MAP_H

#include <common.h>
#include <mappoint.h>
#include <frame.h>

namespace demo {
    class Map {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Map> Ptr;
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyFrameType;

        Map() {}

        //增加关键帧
        void InsertKeyFrame(Frame::Ptr frame);

        //增加地图点
        void InsertMapPoint(MapPoint::Ptr mappoint);

        //获取所有地图点
        LandmarksType GetAllMapPoint() {
          std::unique_lock<std::mutex> lck(data_mutex_);
          return landmarks_;
        }

        //获取所有关键帧
        KeyFrameType GetAllKeyFrame() {
          std::unique_lock<std::mutex> lck(data_mutex_);
          return keyframes_;
        }

        //获取激活的路标点
        LandmarksType GetActiveMapPoint() {
          std::unique_lock<std::mutex> lck(data_mutex_);
          return active_landmarks_;
        }

        //获取激活的关键帧
        KeyFrameType GetActiveKeyFrame() {
          std::unique_lock<std::mutex> lck(data_mutex_);
          return active_keyframe_;
        }
        void CleanMap();
    private:
        //将就得关键帧置为不活跃状态
        void RemoveOldKeyframe();

        std::mutex data_mutex_;
        LandmarksType landmarks_;
        LandmarksType active_landmarks_;
        KeyFrameType keyframes_;
        KeyFrameType active_keyframe_;
        Frame::Ptr current_frame_ = nullptr;
        //settings 激活关键帧的数量
        int num_active_keyframe_ = 7;
    };
} // namespace demo

#endif