#pragma once
#ifndef DEMO_FRAME_H
#define DEMO_FRAME_H

#include <common.h>

namespace demo {
    struct MapPoint;
    struct Feature;

/**
 * ding帧
 * 每一帧分配id，关键帧分配关键帧id
 *
 */
    struct Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Frame> Ptr;
        unsigned long id_ = 0;          //这个帧的id
        unsigned long keyframe_id_ = 0; //这个关键帧的id
        bool is_keyframe_ = false;      //是否为关键帧
        double timestamp_;              //时间戳
        Sophus::SE3d pose_;              // Tcw形式Pose
        std::mutex pose_mutex_;         // pose数据锁
        cv::Mat left_img_, right_img_;  //双目计算机

        std::vector<std::shared_ptr<Feature>> feature_left_;  //左边的特征点
        std::vector<std::shared_ptr<Feature>> feature_right_; //右边的特征点

    public:
        Frame() {}

        Frame(long id, double timestamp, const Sophus::SE3d &pose, const cv::Mat &left,
              const cv::Mat &right);

        Sophus::SE3d Pose() {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            return pose_;
        }

        void SetPose(const Sophus::SE3d &pose) {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            pose_ = pose;
        }

        void SetKeyFrame();

        static std::shared_ptr<Frame> CreateFrame();
    };

} // namespace demo

#endif