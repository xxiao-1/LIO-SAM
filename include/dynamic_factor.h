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

namespace gtsam{
    class DynamicsBase
    {
        friend class dynamicsFactor;
    protected:
        Eigen::Vector3d vel0, omiga0;
        double dt;
        Eigen::Matrix<double, 6, 6> jacobian, covariance;

        double sum_dt;
        Eigen::Vector3d delta_p;
        Eigen::Quaterniond delta_q;
        Eigen::Vector3d delta_v;
        Eigen::Matrix<double, 6, 6> noise;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        DynamicsBase() = delete;
        DynamicsBase(const Eigen::Vector3d &_vel0, const Eigen::Vector3d &_omiga0)//构造函数
                : vel0{ _vel0 }, omiga0{ _omiga0 },dt{0.0},
                  jacobian{ Eigen::Matrix<double, 6, 6>::Identity() }, covariance{ Eigen::Matrix<double, 6, 6>::Zero() },
                  sum_dt{ 0.0 }, delta_p{ Eigen::Vector3d::Zero() }, delta_q{ Eigen::Quaterniond::Identity() }, delta_v{ Eigen::Vector3d::Zero() }

        {
            noise = Eigen::Matrix<double, 6, 6>::Zero();
            noise.block<3, 3>(0, 0) = (0.1 * 0.1) * Eigen::Matrix3d::Identity();
            noise.block<3, 3>(3, 3) = (1 * 1) * Eigen::Matrix3d::Identity();
        }
        virtual ~DynamicsBase() {
        }
        /// print
        void print(const std::string& s = "DynamicsBase Measurements:") ;

        /// equals
        bool equals(const DynamicsBase& expected, double tol = 1e-9) const;

        /// Re-initialize PreintegratedIMUMeasurements
        void resetIntegration() ;

        void push_back(double _dt, const Eigen::Vector3d &_vel0, const Eigen::Vector3d &_omiga0)
        {
            dt = _dt;
            vel0 = _vel0;
            omiga0 = _omiga0;
            Vector3d result_delta_p;
            Quaterniond result_delta_q;
            Vector3d result_delta_v;
            Vector3d result_linearized_ba;
            Vector3d result_linearized_bg;

            dynamics_Integration(_dt,vel0, omiga0, delta_p, delta_q, delta_v,
                                 result_delta_p, result_delta_q, result_delta_v, 1);

            delta_p = result_delta_p;
            delta_q = result_delta_q;
            delta_v = result_delta_v;
            delta_q.normalize();
            sum_dt += dt;
        }
        void dynamics_Integration(double _dt,
                                  const Eigen::Vector3d &_vel0, const Eigen::Vector3d &_omiga0,
                                  const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                                  Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                                  bool update_jacobian);

        //残差项
        Eigen::Matrix<double, 6, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj);

    };

    class dynamicsFactor: public NoiseModelFactor2< Pose3 ,Pose3> {

    private:
        typedef dynamicsFactor This;
        typedef NoiseModelFactor2<Pose3,Pose3> Base;

        DynamicsBase _DB_; ///< Position measurement in cartesian coordinates

    public:

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<dynamicsFactor> shared_ptr;

    /** Default constructor - only use for serialization */
    dynamicsFactor(Key pose_i,  Key pose_j, const DynamicsBase& dynamicsBase);
    virtual ~dynamicsFactor() {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override;

    /// @name Testable
    /// @{
    ///operator
    GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const dynamicsFactor&);
    ///print
    void print(const std::string& s = "", const KeyFormatter& keyFormatter =
    DefaultKeyFormatter) const override;
    ///equals
    bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;
    /// @}

    const Eigen::Matrix<double, 6, 6>&  covariance() const {
        return _DB_.covariance;
    }

/** implement functions needed to derive from Factor */

/// vector of errors
    Vector evaluateError(const Pose3& pose_i, const Pose3& pose_j,boost::optional<Matrix&> H1 =boost::none, boost::optional<Matrix&> H2 = boost::none) const override;
    DynamicsBase* dynamics_integration;
};
};

