#include "common.h"

namespace demo{
    class Config{
    public:
        ~Config();
        static bool SetParameterFile(const std::string& filename);

        template<typename T>
        static T Get(const std::string& key){
            return T(config_->file_[key]);
        }
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;
    };

}