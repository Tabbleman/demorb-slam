#include "common.h"
namespace demo{
inline bool triangulation(const std::vector<Sophus::SE3d>&poses,
                          const std::vector<Vec3>&points, Vec3 &pt_world){
    MatXX A(poses.size() * 2, 4);
    VecX b(poses.size() * 2);
    b.setZero();
    for(int i = 0; i < poses.size(); i ++){
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(i * 2, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(i * 2 + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU|Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3,3)).head<3>();

    if(svd.singularValues()[3] / svd.singularValues()[2] < 1e-2){
        return true;
    }
    return false;
}
inline Vec2 cvPoint2Vec2(const cv::Point2f& p){
    return Vec2(p.x, p.y);
}
}