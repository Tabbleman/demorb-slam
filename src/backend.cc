#include "backend.h"
#include "utils.h"
#include "feature.h"
#include "mappoint.h"
#include "map.h"
#include "g2o_types.h"

namespace demo{
    Backend::Backend() {
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
    }


    void Backend::BackendLoop() {
        while (backend_running_.load()){
            std::unique_lock<std::mutex> lock(data_mutex_);
            map_update_.wait(lock);
            Map::KeyFrameType  active_kfs = map_->GetActiveKeyFrame();
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
        std::map<unsigned long, VertexPose* > vertices;
        unsigned long max_kf_id = 0;
        for(auto keyframe: keyframes){
            auto kf = keyframe.second;
            VertexPose *vertexPose = new VertexPose();
            vertexPose->setId(kf->keyframe_id_);
            vertexPose->setEstimate(kf->Pose());
            optimizer.addVertex(vertexPose);
            if(kf->keyframe_id_ > max_kf_id){
                max_kf_id = kf->keyframe_id_;
            }

            vertices.insert({kf->keyframe_id_, vertexPose});
        }

        std::map<unsigned long, VertexXYZ> vertices_landmarks;
        for(auto landmark: landmarks){
            auto mapPoint = landmark.second;
            if(mapPoint->is_outlier_)continue;

            VertexXYZ *vertexXyz = new VertexXYZ();
            vertexXyz->setId(mapPoint->id_);
            vertexXyz->setEstimate(mapPoint->Pos());

        }

    }
}