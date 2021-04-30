//
// Created by xxiao on 2020/11/17.
//

#ifndef GTSAM_DYNAMICSFACTOR_H
#define GTSAM_DYNAMICSFACTOR_H

#endif //GTSAM_DYNAMICSFACTOR_H
/* GTSAM includes */
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/debug.h>
#include <gtsam/geometry/Pose3.h>

using namespace Eigen;
using namespace std;
//using namespace gtsam;
namespace gtsam {
    class DynamicsBase {
        friend class dynamicsFactor;

    protected:
        double dt;
        Eigen::Vector3d vel0, omiga0;
        Eigen::Matrix<double, 6, 6> jacobian, covariance;
        double sum_dt;
        Eigen::Vector3d delta_p;
        Eigen::Quaterniond delta_q;
        Eigen::Vector3d delta_v;
        Eigen::Matrix<double, 6, 6> noise;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        DynamicsBase() = delete;

        DynamicsBase(double _dt)//构造函数
                : dt{_dt}, vel0{Eigen::Vector3d::Zero()}, omiga0{Eigen::Vector3d::Zero()},
                  jacobian{Eigen::Matrix<double, 6, 6>::Identity()}, covariance{Eigen::Matrix<double, 6, 6>::Zero()},
                  sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()},
                  delta_v{Eigen::Vector3d::Zero()} {
            noise = Eigen::Matrix<double, 6, 6>::Zero();
            noise.block<3, 3>(0, 0) = (0.1 * 0.1) * Eigen::Matrix3d::Identity();
            noise.block<3, 3>(3, 3) = (1 * 1) * Eigen::Matrix3d::Identity();
        }

        virtual ~DynamicsBase() {
        }

        void resetIntegration();

        void push_back(double _dt, const Eigen::Vector3d &_vel0, const Eigen::Vector3d &_omiga0) {
            dt = _dt;
            vel0 = _vel0;
            omiga0 = _omiga0;
            Vector3d result_delta_p;
            Quaterniond result_delta_q;
            Vector3d result_delta_v;
            Vector3d result_linearized_ba;
            Vector3d result_linearized_bg;
            // TODO 输入此时的角速度
            Vector3d un_vel0 = delta_q * _vel0;
            result_delta_q = delta_q * Quaterniond(1, _omiga0(0) * _dt / 2, _omiga0(1) * _dt / 2, _omiga0(2) * _dt / 2);
            result_delta_p = delta_p + un_vel0 *_dt;

            if (1)
            {
                Vector3d w_x = _omiga0;
                Vector3d v_0_x = _vel0;
                Matrix3d R_w_x, R_v_0_x;

                //欧拉角到旋转矩阵
                R_w_x << 0, -w_x(2), w_x(1),
                        w_x(2), 0, -w_x(0),
                        -w_x(1), w_x(0), 0;
                R_v_0_x << 0, -v_0_x(2), v_0_x(1),
                        v_0_x(2), 0, -v_0_x(0),
                        -v_0_x(1), v_0_x(0), 0;


                MatrixXd F = MatrixXd::Zero(6, 6);
                F.block<3, 3>(3, 3) = Matrix3d::Identity();
                F.block<3, 3>(3, 0) = -delta_q.toRotationMatrix()*R_v_0_x *_dt;
                F.block<3, 3>(0, 0) = Matrix3d::Identity() - R_w_x * _dt;

                MatrixXd V = MatrixXd::Zero(6, 6);
                V.block<3, 3>(3 ,3) = delta_q.toRotationMatrix() * _dt;
                V.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3) * _dt;

                jacobian = F * jacobian;
                covariance = F * covariance * F.transpose() + V * noise * V.transpose();
            }


            delta_p = result_delta_p;
            delta_q = result_delta_q;
            delta_v = result_delta_v;
            delta_q.normalize();
        }

        void showDelt() {
            std::cout << "delta_p=" << delta_p << " delta_v=" << delta_v << std::endl;
        }

        Vector3d getDeltaP() {
            return delta_p;
        }
        Quaterniond getDeltaQ(){
            return delta_q;
        }
        Eigen::Matrix<double, 6, 6> getCovariance(){
            return covariance;
        }


        void dynamics_Integration(double _dt,
                                  const Eigen::Vector3d &_vel0, const Eigen::Vector3d &_omiga0,
                                  const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q,
                                  const Eigen::Vector3d &delta_v,
                                  Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q,
                                  Eigen::Vector3d &result_delta_v,
                                  bool update_jacobian);

        //残差项
        Eigen::Matrix<double, 6, 1>
        evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Pj,
                 const Eigen::Quaterniond &Qj);

    };

    class dynamicsFactor : public gtsam::NoiseModelFactor2<Pose3, Pose3> {

    private:
        typedef dynamicsFactor This;
        typedef NoiseModelFactor2 <Pose3, Pose3> Base;

        DynamicsBase _DB_; ///< Position measurement in cartesian coordinates

    public:

        /// shorthand for a smart pointer to a factor
        typedef boost::shared_ptr <dynamicsFactor> shared_ptr;

        /** Default constructor - only use for serialization */
        dynamicsFactor(Key pose_i, Key pose_j, const DynamicsBase &dynamicsBase);

        virtual ~dynamicsFactor() {}

        /// @return a deep copy of this factor
        gtsam::NonlinearFactor::shared_ptr clone() const override;

        /// @name Testable
        /// @{
        ///operator
        GTSAM_EXPORT friend std::ostream
        &

        operator<<(std::ostream &os, const dynamicsFactor &);

        ///print
        void print(const std::string &s = "", const KeyFormatter &keyFormatter =
        DefaultKeyFormatter) const override;

        ///equals
        bool equals(const NonlinearFactor &expected, double tol = 1e-9) const override;
        /// @}

        const Eigen::Matrix<double, 6, 6> &covariance() const {
            return _DB_.covariance;
        }

/** implement functions needed to derive from Factor */

/// vector of errors
        Vector evaluateError(const Pose3 &pose_i, const Pose3 &pose_j, boost::optional<Matrix &> H1 = boost::none,
                             boost::optional<Matrix &> H2 = boost::none);

        DynamicsBase *dynamics_integration;
    };

}