#include "frontend.h"
#include "feature.h"
#include "map.h"

namespace demo{
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
        if (last_frame_){
            current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
        }
        int num_track_last = TrackLastFrame();
        tracking_inliers_ = EstimateCurrentPose();
        if(num_track_last > num_features_tracking_good_) {
            status_ = FrontendStatus::TRACKING_GOOD;
        }else if (num_track_last > num_features_tracking_bad_){
            status_ = FrontendStatus::TRACKING_BAD;
        }else {
            status_ = FrontendStatus::LOST;
        }

        InsertKeyFrame();
        relative_motion_ = current_frame_->Pose() * last_frame_->Pose();
        if(viewer_){

        }
        return true;

    }

    bool Frontend::InsertKeyFrame() {
        if(tracking_inliers_ >= num_features_needed_for_keyframes){
            //enough point, doesn't need insert kf
            return  false;
        }
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);

        LOG(INFO) << "Set keyframe" << current_frame_->id_ << "as "
         << current_frame_->keyframe_id_ ;
        SetObservationsForKeyFrame();
        DetectFeatures();

        FindFeaturesInFight();
        TriangulateNewPoints();
//      后端更新地图
//      Viwer更新
        return true;
    }

    void Frontend::SetObservationsForKeyFrame() {
        for(auto &feat: current_frame_->feature_left_){
            auto mp = feat->map_point_.lock();
            if(mp) mp->AddObservation(feat);
        }
    }

    int Frontend::TriangulateNewPoints() {
        std::vector<Sophus::SE3d> poses{camera_left_->Pose(), camera_right_->Pose()};
        Sophus::SE3d current_pose_Tcw = current_frame_->Pose().inverse();
        int cnt_triangulated_points = 0;
        for(int i = 0; i < current_frame_->feature_left_.size(); i ++){
            // 左边的点没有关联mappoint并且存在右图匹配点，尝试三角化
            if(current_frame_->feature_left_[i]->map_point_.expired() &&
                current_frame_->feature_right_[i] != nullptr){
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
                if(demo::triangulation(poses, points, pworld) && pworld[2] > 0){
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
//        typedef
    }

//    Frontend::
}