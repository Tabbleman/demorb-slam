#pragma once
#ifndef DEMO_FEATURE_H
#define DEMO_FEATURE_H

#include <common.h>
namespace demo {
    struct Frame;
    struct MapPoint;

    struct Feature {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef std::shared_ptr<Feature> Ptr;

        std::weak_ptr<Frame> frame_;        //持有该特征的帧，由于避免智能指针的相互引用，使用weak_ptr
        cv::KeyPoint position_;             // 2d提取位置
        std::weak_ptr<MapPoint> map_point_; //关联地图点

        bool is_outlier_ = false;           //是否是异常点
        bool is_on_left_img_ = true;        //表示是否在左边，false表示在右边
    public:
        Feature() {}

        Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
            : frame_(frame), position_(kp) {}
    };

} // namespace demo

#endif

