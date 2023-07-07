#include "backend.h"
#include "utils.h"
#include "feature.h"
#include "mappoint.h"
#include "map.h"
#include "g2o_types.h"

namespace demo {
    Backend::Backend() {
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
    }

    void Backend::UpdateMap() {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.notify_one();
    }

    void Backend::Stop() {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.notify_one();
        backend_thread_.join();
    }

    void Backend::BackendLoop() {
        while (backend_running_.load()) {
            std::unique_lock<std::mutex> lock(data_mutex_);
            map_update_.wait(lock);
            Map::KeyFrameType active_kfs = map_->GetActiveKeyFrame();
            Map::LandmarksType active_landmarks = map_->GetAllMapPoint();
            Optimize(active_kfs, active_landmarks);
        }
    }

    void Backend::Optimize(Map::KeyFrameType &keyframes, Map::LandmarksType &landmarks) {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
                LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>())
        );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        std::map<unsigned long, VertexPose *> vertices;
        unsigned long max_kf_id = 0;
        for (auto keyframe: keyframes) {
            auto kf = keyframe.second;
            VertexPose *vertexPose = new VertexPose();
            vertexPose->setId(kf->keyframe_id_);
            vertexPose->setEstimate(kf->Pose());
            optimizer.addVertex(vertexPose);
            if (kf->keyframe_id_ > max_kf_id) {
                max_kf_id = kf->keyframe_id_;
            }

            vertices.insert({kf->keyframe_id_, vertexPose});
        }

        std::map<unsigned long, VertexXYZ*> vertices_landmarks;

        Mat33 K = camera_left_->K();
        Sophus::SE3d left_ex = camera_left_->Pose();
        Sophus::SE3d right_ex = camera_right_->Pose();

        int index = 1;
        double threshold = 5.0 ;//magic rubust kernel
        std::map<EdgePoseAndMap*, Feature::Ptr> edges_and_features;
        for (auto landmark: landmarks) {
            if (landmark.second->is_outlier_)continue;
            unsigned long landmark_id = landmark.second->id_;

            auto observations = landmark.second->GetObs();
            for (auto &obs: observations) {
                if (obs.lock() == nullptr)continue;
                auto feat = obs.lock();
                if (feat->is_outlier_ || feat->frame_.lock() == nullptr)continue;
                auto frame = feat->frame_.lock();
                EdgePoseAndMap *edge = nullptr;
                if (feat->is_on_left_img_) edge = new EdgePoseAndMap(K, left_ex);
                else edge = new EdgePoseAndMap(K, right_ex);
                /**
                 * 路标没在待优化的里面，就加一个顶点
                 */
                if (vertices_landmarks.find(landmark_id) == vertices_landmarks.end()) {
                    VertexXYZ *v = new VertexXYZ;
                    v->setEstimate(landmark.second->Pos());
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true);
                    vertices_landmarks.insert({landmark_id, v});
                    optimizer.addVertex(v);

                }
                edge->setId(index);
                edge->setVertex(0, vertices.at(frame->keyframe_id_));
                edge->setVertex(1, vertices_landmarks.at(landmark_id));
                edge->setMeasurement(cvPoint2Vec2(feat->position_.pt));
                edge->setInformation(Mat22::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(threshold);
                edge->setRobustKernel(rk);
                edges_and_features.insert({edge, feat});
                optimizer.addEdge(edge);
                index++;
            }
        }
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        int cnt_inlier = 0, cnt_outlier = 0;
        int iteration = 0;
        while(iteration < 5){
            cnt_inlier = 0;
            cnt_outlier = 0;

            for(auto &edge_and_feature: edges_and_features){
                if(edge_and_feature.first->chi2() > threshold){
                    cnt_outlier ++;
                }else {
                    cnt_inlier ++;
                }
            }
            double inlier_ratio = cnt_inlier / (double)(cnt_inlier + cnt_outlier);
            if(inlier_ratio > 0.5) {
                break;
            }else {
                threshold *= 2;
                iteration++;
            }

        }
        for(auto &edge_and_feature: edges_and_features){
            if(edge_and_feature.first->chi2() > threshold){
                edge_and_feature.second->is_outlier_ = true;

                edge_and_feature.second->map_point_.lock()->RemoveObservation(edge_and_feature.second);
            }else {
                edge_and_feature.second->is_outlier_ = false;
            }
        }
        LOG(INFO) << "Outlier / Inlier: " << cnt_outlier << '/' << cnt_inlier ;
        for(auto &v: vertices){
            keyframes.at(v.first)->SetPose(v.second->estimate());
        }
        for(auto &v: vertices_landmarks){
            landmarks.at(v.first)->SetPos(v.second->estimate());

        }
    }
}