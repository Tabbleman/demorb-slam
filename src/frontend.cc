#include "frontend.h"
namespace demo{
    bool Frontend::AddFrame(Frame::Ptr frame) {
        currenf_frame_ = frame;
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
        last_frame_ = currenf_frame_;
        return true;
    }

    bool Frontend::Track() {
        if (last_frame_){
            currenf_frame_->SetPose(relative_motion_ * last_frame_->Pose());
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
        relative_motion_ = currenf_frame_->Pose() * last_frame_->Pose();
        if(viewer_){

        }
        return true;

    }

    bool Frontend::InsertKeyFrame() {
        if(tracking_inliers_ >= num_features_needed_for_keyframes){
            //enough point, doesn't need insert kf
            return  false;
        }
        currenf_frame_->SetKeyFrame();
        map_->InsertKeyFrame(currenf_frame_);

        LOG(INFO) << "Set keyframe" << currenf_frame_->id_ << "as "
         << currenf_frame_->keyframe_id_ ;
        SetObservationsForKeyFrame();
        DetectFeatures();

        FindFeaturesInFight();
        TriangulateNewPoints();
//      后端更新地图
//      Viwer更新
        return true;
    }

    void Frontend::SetObservationsForKeyFrame() {
        Feature::Ptr feat ;
        for(auto &feat: currenf_frame_->feature_left_){

        }
    }

}