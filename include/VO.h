#pragma once

#include "common.h"
#include "frontend.h"
#include "backend.h"
#include "viewer.h"
#include "dataset.h"
#include "config.h"

namespace demo{
    class VO{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<VO> Ptr;
        VO(std::string &config_file);
        bool Init();

        void Run();

        bool Step();

        FrontendStatus GetFrontEndStatus(){return frontend_->GetFrontEndStatus();}

    private:
        bool init_ = false;
        std::string config_file_path_;
        /**
         * vo由前端和后端组成，前端负责提取特征点，
         * 判断是否要交给后端进行优化，
         * loop closure and loop detect 也要加入
         * 到前端里面，检测到loop就交给后端优化
         * Viewer要开一个线程
         */
        Frontend::Ptr frontend_ = nullptr;
        Backend::Ptr  backend_ = nullptr;
        Map::Ptr map_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;
        Dataset::Ptr dataset_ = nullptr;
    };
}