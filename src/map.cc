#include <map.h>
#include "feature.h"

namespace demo {
    void Map::InsertKeyFrame(Frame::Ptr frame) {
        current_frame_ = frame;
        if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
            keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
            active_keyframe_.insert(std::make_pair(frame->keyframe_id_, frame));
        } else {
            keyframes_[frame->keyframe_id_] = frame;
            active_keyframe_[frame->keyframe_id_] = frame;
        }

        if (active_keyframe_.size() > num_active_keyframe_) {
            RemoveOldKeyframe();
        }
    }

    void Map::InsertMapPoint(MapPoint::Ptr mappoint) {
        if (landmarks_.find(mappoint->id_) == landmarks_.end()) {
            landmarks_.insert(std::make_pair(mappoint->id_, mappoint));
            active_landmarks_.insert(std::make_pair(mappoint->id_, mappoint));
        } else {
            landmarks_[mappoint->id_] = mappoint;
            active_landmarks_[mappoint->id_] = mappoint;
        }
    }

//  删除旧的关键帧
    void Map::RemoveOldKeyframe() {
        if (current_frame_ == nullptr)return;
        double min_dist = 1e5, max_dist = 0;
        double min_kf_id = 0, max_kf_id = 0;
        auto Tcw = current_frame_->Pose().inverse();
        for (auto &kf: active_keyframe_) {
            if (kf.second == current_frame_) continue;
            auto dist = (kf.second->Pose() * Tcw).log().norm();
            if (dist > max_dist) {
                max_dist = dist;
                max_kf_id = kf.first;
            }
            if (dist < min_dist) {
                min_dist = dist;
                min_kf_id = kf.first;
            }

        }
//        最近阈值
        const double min_dist_threshold = 0.3;
        Frame::Ptr frame_to_remove = nullptr;
        if(min_dist < min_dist_threshold) {
            frame_to_remove = keyframes_.at(min_kf_id);
        }else {
            frame_to_remove = keyframes_.at(max_kf_id);
        }
        LOG(INFO) << "remove keyframe" << frame_to_remove->keyframe_id_;
        active_keyframe_.erase(frame_to_remove->keyframe_id_);
        //删除在要删除的关键帧里面所有的特征点,左右都有特征点
        for (auto feat: frame_to_remove->feature_left_) {
            auto mp = feat->map_point_.lock();
            if (mp) {
                mp->RemoveObservation(feat);
            }
        }
        for (auto feat: frame_to_remove->feature_right_) {
            auto mp = feat->map_point_.lock();
            if (mp) {
                mp->RemoveObservation(feat);
            }
        }
        //清理地图
        CleanMap();
    }

    void Map::CleanMap() {
        int cnt_landmark_removed = 0;
        for (auto iter = active_landmarks_.begin();
             iter != active_landmarks_.end();
                ) {
            //观测次数为0就一处这个点
            if (iter->second->observed_times == 0) {
                iter = active_landmarks_.erase(iter);
                cnt_landmark_removed++;
            } else {
                iter++;
            }
        }
        LOG(INFO) << "Removed " << cnt_landmark_removed;
    }

} // namespace demo
