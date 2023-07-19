#include "frontend.h"
#include "feature.h"
#include "map.h"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "config.h"

namespace demo {
    Frontend::Frontend() {
        orb_ = cv::ORB::create();
        gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
        num_features_init = Config::Get<int>("num_feature_init");
        num_features_ = Config::Get<int>("num_features");

    }

    bool Frontend::AddFrame(Frame::Ptr frame) {
        current_frame_ = frame;
        switch (status_) {
            case FrontendStatus::INITING:
                StereoInit();
                break;
            case FrontendStatus::TRACKING_GOOD:
            case FrontendStatus::TRACKING_BAD:
                Track();
                break;
            case FrontendStatus::LOST:
                Reset();
                break;
        }
        last_frame_ = current_frame_;
        return true;
    }

    bool Frontend::Track() {
        if (last_frame_) {
            current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
        }
        int num_track_last = TrackLastFrameLK();
        tracking_inliers_ = EstimateCurrentPose();
        if (num_track_last > num_features_tracking_good_) {
            status_ = FrontendStatus::TRACKING_GOOD;
        } else if (num_track_last > num_features_tracking_bad_) {
            status_ = FrontendStatus::TRACKING_BAD;
        } else {
            status_ = FrontendStatus::LOST;
        }

        InsertKeyFrame();
        relative_motion_ = current_frame_->Pose() * last_frame_->Pose();
        if (viewer_) {

        }
        return true;

    }

    bool Frontend::InsertKeyFrame() {
        if (tracking_inliers_ >= num_features_needed_for_keyframes) {
            //enough point, doesn't need insert kf
            return false;
        }
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);

        LOG(INFO) << "Set keyframe" << current_frame_->id_ << "as "
                  << current_frame_->keyframe_id_;
        SetObservationsForKeyFrame();
        DetectFeaturesGFTT();

        FindFeaturesInRightLK();
        TriangulateNewPoints();
//      后端更新地图
//      Viwer更新
        return true;
    }

    void Frontend::SetObservationsForKeyFrame() {
        for (auto &feat: current_frame_->feature_left_) {
            auto mp = feat->map_point_.lock();
            if (mp) mp->AddObservation(feat);
        }
    }

    int Frontend::TriangulateNewPoints() {
        std::vector<Sophus::SE3d> poses{camera_left_->Pose(), camera_right_->Pose()};
        Sophus::SE3d current_pose_Tcw = current_frame_->Pose().inverse();
        int cnt_triangulated_points = 0;
        for (int i = 0; i < current_frame_->feature_left_.size(); i++) {
            // 左边的点没有关联mappoint并且存在右图匹配点，尝试三角化
            if (current_frame_->feature_left_[i]->map_point_.expired() &&
                current_frame_->feature_right_[i] != nullptr) {
                std::vector<Vec3> points{
                        camera_left_->pixel2camera(
                                Vec2(current_frame_->feature_left_[i]->position_.pt.x,
                                     current_frame_->feature_left_[i]->position_.pt.y
                                )
                        ),
                        camera_right_->pixel2camera(
                                Vec2(current_frame_->feature_right_[i]->position_.pt.x,
                                     current_frame_->feature_right_[i]->position_.pt.y)
                        )
                };

                Vec3 pworld = Vec3::Zero();
                if (demo::triangulation(poses, points, pworld) && pworld[2] > 0) {
                    auto new_map_point = MapPoint::CreateNewMapPoint();
                    pworld = current_pose_Tcw * pworld;
                    new_map_point->SetPos(pworld);
                    new_map_point->AddObservation(current_frame_->feature_left_[i]);
                    new_map_point->AddObservation(current_frame_->feature_right_[i]);
                    current_frame_->feature_left_[i]->map_point_ = new_map_point;
                    current_frame_->feature_right_[i]->map_point_ = new_map_point;
                    map_->InsertMapPoint(new_map_point);
                    cnt_triangulated_points++;
                }
            }
        }
        LOG(INFO) << "new landmarks: " << cnt_triangulated_points;
        return cnt_triangulated_points;
    }

    int Frontend::EstimateCurrentPose() {
//        TODO

    }

    /**
     * 跟踪上一帧
     * @return
     */
    int Frontend::TrackLastFrameORB() {
        //orb extractor
        //start initialization
        int num_good_pts = 0;
        std::vector<cv::KeyPoint> keypoint_last, keypoint_current;

        cv::Mat descriptor_last, descriptor_current;

        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
        cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

        detector->detect(last_frame_->left_img_, keypoint_last);
        detector->detect(current_frame_->left_img_, keypoint_current);

        descriptor->compute(last_frame_->left_img_, keypoint_last, descriptor_last);
        descriptor->compute(current_frame_->left_img_, keypoint_current, descriptor_current);

        std::vector<cv::DMatch> matches;
        matcher->match(descriptor_last, descriptor_current, matches);
        auto cmp = [](const cv::DMatch &dm1, const cv::DMatch &dm2) { return dm1.distance < dm2.distance; };
        auto min_max = std::minmax_element(matches.begin(), matches.end(), cmp);
        double min_dist = min_max.first->distance;
        double max_dist = min_max.second->distance;

        for (int i = 0; i < descriptor_last.rows; i++) {
            cv::KeyPoint current_good = keypoint_current[matches[i].trainIdx];
            if (matches[i].distance < std::max(min_dist * 2, 10.0)) {
                LOG(INFO) << "Good Match Found!";
                demo::Feature::Ptr feature(new Feature(current_frame_, current_good));
                feature->map_point_ = last_frame_->feature_left_[i]->map_point_;
                current_frame_->feature_left_.push_back(feature);
                num_good_pts++;
            }
        }
        LOG(INFO) << "Found " << num_good_pts << " Good in the last image";
        return num_good_pts;
    }

    int Frontend::TrackLastFrameLK() {
        std::vector<cv::Point2f> kp_last, kp_currrent;
        for (auto &kp: last_frame_->feature_left_) {
            if (kp->map_point_.lock()) {
                auto mp = kp->map_point_.lock();
                auto px =
                        camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
                kp_last.push_back(kp->position_.pt);
                kp_currrent.push_back(cv::Point2f(px[0], px[1]));

            } else {
                kp_last.push_back(kp->position_.pt);
                kp_currrent.push_back(kp->position_.pt);
            }
        }
        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(last_frame_->left_img_, current_frame_->left_img_,
                                 kp_last, kp_currrent,
                                 status, error, cv::Size(11, 11),
                                 3, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                 cv::OPTFLOW_FARNEBACK_GAUSSIAN
                                 );
        int num_good_pts = 0;
        for(int i = 0; i < status.size(); i ++){
            if(status[i]){
                cv::KeyPoint kp(kp_currrent[i], 7);
                Feature::Ptr feature(new Feature(current_frame_, kp));
                feature->map_point_ = last_frame_->feature_left_[i]->map_point_;
                current_frame_->feature_left_.push_back(feature);
                num_good_pts++;
            }

        }
        LOG(INFO) << "Good point found: " << num_good_pts ;
        return num_good_pts;
    }

    bool Frontend::StereoInit() {
        int num_feature_left = DetectFeaturesGFTT();
        int num_coor_feature = FindFeaturesInRightLK();
        if (num_coor_feature < num_features_init) {
            return false;
        }

        bool build_map_success = BuildInitMap();
        if (build_map_success) {
            status_ = FrontendStatus::TRACKING_GOOD;
            if (viewer_) {
//                TODO
            }
            return true;
        }
        return false;

    }
    /**
     * 用fast检测特征点
     * @return
     */
    int Frontend::DetectFeaturesFast() {
//      TODO
    }

    int Frontend::DetectFeaturesORB() {
//        TODO
    }

    int Frontend::DetectFeaturesGFTT() {
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 200);
        for(auto &feat: current_frame_->feature_left_){
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                          feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
        }
        int cnt_detect = 0;
        std::vector<cv::KeyPoint> keypoints;
        gftt_->detect(current_frame_->left_img_, keypoints, mask);
        for(auto &kp: keypoints){
            current_frame_->feature_left_.push_back(
                    Feature::Ptr(new Feature(current_frame_, kp))
                    );
            cnt_detect ++;

        }
        LOG(INFO) << "Detect " << cnt_detect << " Features";
        return cnt_detect;
    }

    int Frontend::FindFeaturesInRightLK() {

    }
    bool Frontend::BuildInitMap() {
        std::vector<Sophus::SE3d> poses{
                camera_left_->Pose(), camera_right_->Pose()
        };
        int cnt_init_landmarks = 0;
        for (int i = 0; i < current_frame_->feature_left_.size(); i++) {
            std::vector<Vec3> points{
                    camera_left_->pixel2camera(
                            Vec2(current_frame_->feature_left_[i]->position_.pt.x,
                                 current_frame_->feature_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(Vec2(current_frame_->feature_right_[i]->position_.pt.x,
                                                     current_frame_->feature_right_[i]->position_.pt.y))
            };
            Vec3 pworld = Vec3::Zero();

            if(triangulation(poses, points, pworld) && pworld[2] > 0){
                auto new_map_point = MapPoint::CreateNewMapPoint();
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(current_frame_->feature_left_[i]);
                new_map_point->AddObservation(current_frame_->feature_right_[i]);
                cnt_init_landmarks ++;
                map_->InsertMapPoint(new_map_point);

            }

        }
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);
//        backend update
        backend_->UpdateMap();

    }


    bool Frontend::Reset() {
        return true;
    }

}