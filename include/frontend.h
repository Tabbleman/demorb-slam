#pragma once

#include "common.h"
#include "opencv2/features2d.hpp"
#include "frame.h"
#include "map.h"
#include "utils.h"
namespace demo {
    class Backend;
    class Viewer;

    enum class FrontendStatus {
        INITING, TRACKING_GOOD, TRACKING_BAD, LOST
    };

    class Frontend {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Frontend> Ptr;

        Frontend();

        //添加一帧，并确定计算定位结果
        bool AddFrame(Frame::Ptr frame);

        //set map
        void SetMap(Map::Ptr map) {map_ = map; }

        void SetBackend(std::shared_ptr<Backend> backend){backend_ = backend;}
        void SetCamera(Camera::Ptr left, Camera::Ptr right){camera_left_ = left, camera_right_ = right;}
    private:
        /**
         * track in normal mode
         * @return true if success
         */
        bool Track();

        /**
         * reset when lost
         * @return true if success
         */
        bool Reset();

        /**
         * Track with last frame
         * @return number if tracked point
         */
        int TrackLastFrame();

        /**
         * estimate current pose
         * @return number of inlier
         */
        int EstimateCurrentPose();

        /**
         * set currnt frame as a key frame and insert it into backend
         * @return true if success
         */
        bool InsertKeyFrame();

        /**
         * try init the front end with stereo images saved in current frame;
         * @return true if success;
         */
        bool StereoInit();

        /**
         * detect features in left image;
         * @return number of features found
         */
        int DetectFeatures();

        /**
         * Find corresponding features in right image of current frame;
         * @return num of features found
         */
        int FindFeaturesInRight();


        /**
         * build the initial map with single image;
         * @return true if success;
         */
        bool BuildInitMap();

        /**
         * Triangulate the 2D points in current frame
         * @return 2d points in current frame
         */
        int TriangulateNewPoints();

        /**
         * 把关键帧的天真点设置成地图点的一个观测值
         */
        void SetObservationsForKeyFrame();

        //data
        FrontendStatus status_ = FrontendStatus::INITING;

        Frame::Ptr current_frame_ = nullptr;
        Frame::Ptr last_frame_ = nullptr;
        Camera::Ptr camera_left_ = nullptr;
        Camera::Ptr camera_right_ = nullptr;

        Map::Ptr map_ = nullptr;
        std::shared_ptr<Backend> backend_ = nullptr;
        std::shared_ptr<Viewer> viewer_ = nullptr;

        Sophus::SE3d relative_motion_;

        int tracking_inliers_ = 0;

        int num_features_ = 150;
        int num_features_init = 0;
        int num_features_tracking_good_ = 50;
        int num_features_tracking_bad_ = 20;
        int num_features_needed_for_keyframes = 0;
        //parameters
        cv::Ptr<cv::FastFeatureDetector> ff_;

    };

}