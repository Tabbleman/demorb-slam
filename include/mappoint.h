#pragma once
#ifndef DEMO_MAPPOINT_H
#define DEMO_MAPPOINT_H

#include <common.h>

namespace demo {
    struct Frame;
    struct Feature;

    struct MapPoint {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<MapPoint> Ptr;

        unsigned long id_ = 0;
        bool is_outlier_ = false;
        Vec3 pos_ = Vec3::Zero();   //position in world
        std::mutex data_mutex_;
        int observed_times = 0;
        std::list<std::weak_ptr<Feature>> observations_;

    public:
        MapPoint() {}

        MapPoint(long id, Vec3 position);

        Vec3 Pos() {
          std::unique_lock<std::mutex> lck(data_mutex_);
          return pos_;
        }

        void SetPos(const Vec3 &pos) {
          std::unique_lock<std::mutex> lck(data_mutex_);
          pos_ = pos;
        }

        void AddObservation(std::shared_ptr<Feature> feature) {
          std::unique_lock<std::mutex> lck(data_mutex_);
          observations_.push_back(feature);
          observed_times++;
        }

        void RemoveObservation(std::shared_ptr<Feature> feature);

        std::list<std::weak_ptr<Feature>> GetObs() {
          std::unique_lock<std::mutex> lck(data_mutex_);
          return observations_;
        }

        static MapPoint::Ptr CreateNewMapPoint();//工厂函数
    };


} // namespace demo


#endif