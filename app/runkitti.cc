#include <iostream>
#include "VO.h"
using namespace std;

DEFINE_string(config_string, "../config/kitti.yml", "config file path");
int main(int argc, char** argv){
    google::ParseCommandLineFlags(&argc, &argv, true);
    demo::VO::Ptr vo(new demo::VO(FLAGS_config_string));
    assert(vo->Init());
    vo->Run();

    return 0;
}
