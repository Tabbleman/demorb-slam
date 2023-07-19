//
// Created by z on 7/7/23.
//
#pragma once
#ifndef DEMOORB_VIEWER_H
#define DEMOORB_VIEWER_H
#include "common.h"
#include "pangolin/pangolin.h"
#include "frame.h"
#include "feature.h"
#include "map.h"

namespace demo{
    class Viewer{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Viewer> Ptr;
        Viewer();

        void SetMap(Map::Ptr& map){map_ = map;}

        void Close();

        void AddCurrentFraem(Frame::Ptr frame);

        void UpdateMap();
    private:
        void ThreadLoop();

        void DrawFrame(Frame::Ptr frame, const float* color);

        void DrawMapPoints();

        void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

        /**
         * 在当前帧画特征点
         * @return
         */
        cv::Mat PlotFrameImage();
        Frame::Ptr current_frame_ = nullptr;
        Map::Ptr map_ = nullptr;

        std::thread viewer_thread_;
        bool viewer_running_ = false;

        std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
        std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
        bool map_updated_ = false;

        std::mutex viewer_data_mutex_;

    };
}

#endif //DEMOORB_VIEWER_H
