#include "common.h"

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/linear_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
namespace demo {
    /**
     * 位姿定点
     */
    class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual void setToOriginImpl() override { _estimate = Sophus::SE3d(); }

        virtual void oplusImpl(const double *update) override {
            Vec6 update_eigen;
            update_eigen << update[0], update[1], update[2],
                    update[3], update[4], update[5];
            _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
        }

        virtual bool read(std::istream &in) override { return true; }

        virtual bool write(std::ostream &out) const override { return true; }
    };

    /**
     * 路标类
     */
    class VertexXYZ : public g2o::BaseVertex<3, Vec3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual void setToOriginImpl() override { _estimate = Vec3::Zero(); }

        virtual void oplusImpl(const double *update) override {
            _estimate[0] += update[0];
            _estimate[1] += update[1];
            _estimate[2] += update[2];
        }

        virtual bool read(std::istream &is) override {
            return true;
        }

        virtual bool write(std::ostream &os) const override {
            return true;
        }

    };
    class EdgePoseOnly: public g2o::BaseUnaryEdge<2, Vec2, VertexPose>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgePoseOnly(const Vec3& pos, const Mat33& K)
            :_pos3d(pos), _K(K){}
        virtual void computeError() override{
            const VertexPose *v = static_cast<VertexPose*>(_vertices[0]);
            Sophus::SE3d T = v->estimate();
            Vec3 pos_pixel = _K * (T * _pos3d);
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();

        }
        virtual void linearizeOplus() override{
            const VertexPose *v = static_cast<VertexPose*>(_vertices[0]);
            Sophus::SE3d T = v->estimate();
            Vec3 pos_cam = T * _pos3d;
            double fx = _K(0, 0);
            double fy = _K(0, 2);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Zinv = 1 / (Z + 1e-10);
            double Zinv2 = Zinv * Zinv;
            _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                    -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                    fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                    -fy * X * Zinv;

        }
        virtual bool write(std::ostream &os) const override{ return true;}
        virtual bool read(std::istream &is) override{return true;}

    private:
        Vec3 _pos3d;
        Mat33 _K;

    };

    class EdgePoseAndMap: g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /**
         * 待优化的变量
         * @param K 内参
         * @param cam 外参
         */
        EdgePoseAndMap(const Mat33 &K, const Sophus::SE3d &cam_ex): _K(K){
            _cam_ex = cam_ex;
        }
        virtual void computeError() override{
            const VertexPose *vertexPose = static_cast<VertexPose*>(_vertices[0]);
            const VertexXYZ *vertexXyz = static_cast<VertexXYZ*>(_vertices[1]);

            Sophus::SE3d T = vertexPose->estimate();
            Vec3 pos_pixel = _K * (_cam_ex * (T * vertexXyz->estimate()));
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();

        }
        virtual void linearizeOplus(g2o::JacobianWorkspace &jacobianWorkspace) override{
            const VertexPose *vertexPose = static_cast<VertexPose*>(_vertices[0]);
            const VertexXYZ *vertexXyz = static_cast<VertexXYZ>(_vertices[1]);

            Sophus::SE3d T = vertexPose->estimate();
            Vec3 pworld = vertexXyz->estimate();

            Vec3 pos_cam = _cam_ex * T *pworld;
            double fx = _K(0, 0);
            double fy = _K(0, 2);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Zinv = 1 / (Z + 1e-10);
            double Zinv2 = Zinv * Zinv;
            _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                    -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                    fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                    -fy * X * Zinv;
            _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *_cam_ex.rotationMatrix()
            * T.rotationMatrix();

        }
        virtual bool read(std::istream &is) override{return true;}
        virtual bool write(std::ostream &os) const override{return true;}
    private:
        Mat33 _K;
        Sophus::SE3d _cam_ex;

    };
}