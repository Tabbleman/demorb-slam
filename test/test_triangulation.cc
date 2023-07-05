#include <gtest/gtest.h>
#include "common.h"
#include "utils.h"

TEST(MyTEST, Triangulation){
    Vec3 pt_world(50, 20, 10), pt_world_estimate;
    std::vector<Sophus::SE3d> poses{
        Sophus::SE3d(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, 0, 0)),
        Sophus::SE3d(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, -10, 0)),
        Sophus::SE3d(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, 10, 0))
    };
    std::vector<Vec3> points;
    for(int i = 0; i < poses.size(); i ++){
        Vec3 pc = poses[i] * pt_world;
        //归一化
        pc /= pc[2];
        points.push_back(pc);
    }
    EXPECT_TRUE(demo::triangulation(poses, points, pt_world_estimate));
    EXPECT_NEAR(pt_world[0], pt_world_estimate[0], 1);
    EXPECT_NEAR(pt_world[1], pt_world_estimate[1], 1);
    EXPECT_NEAR(pt_world[2], pt_world_estimate[2], 1);

}
int main(int argc, char** argv){
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}