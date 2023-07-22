//
// Created by z on 7/10/23.
//
#include "VO.h"

namespace demo {
    VO::VO(std::string &config_file)
            : config_file_path_(config_file) {
    }

    bool VO::Init() {
        if (Config::SetParameterFile(config_file_path_) == false) {
            return false;
        }
        dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
        CHECK_EQ(dataset_->Init(), true);
        LOG(INFO) << "Dataset init successfully";
        //创建物件
        frontend_ = Frontend::Ptr(new Frontend);
        backend_ = Backend::Ptr(new Backend);
        map_ = Map::Ptr(new Map);
        viewer_ = Viewer::Ptr(new Viewer);

        frontend_->SetBackend(backend_);
        frontend_->SetMap(map_);
        frontend_->SetViewer(viewer_);
        frontend_->SetCamera(dataset_->GetCamera(0), dataset_->GetCamera(1));

        backend_->SetMap(map_);
        backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

        viewer_->SetMap(map_);
        return true;
    }

    void VO::Run() {
        while (true) {
            LOG(INFO) << "VO is RUNNING";
            if (Step() == false) {
                break;
            }
        }
        backend_->Stop();
        viewer_->Close();
        LOG(INFO) << "VO EXIT";
    }

    bool VO::Step() {
        Frame::Ptr next_frame = dataset_->NextFrame();
        if(next_frame == nullptr) return false;
        auto st = std::chrono::steady_clock::now();
        bool status = frontend_->AddFrame(next_frame);
        auto ed = std::chrono::steady_clock::now();
        auto delta = std::chrono::duration_cast<std::chrono::duration<double>>(ed - st);
        LOG(INFO) << "VO COST " << delta.count();
        return status;
    }
}
