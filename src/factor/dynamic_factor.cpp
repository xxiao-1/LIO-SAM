//
// Created by xxiao on 2020/11/17.
//
#include "../../include/dynamic_factor.h"
#include <ostream>
#include <gtsam/geometry/Utility.h>

namespace gtsam {
    using namespace std;
    //------------------------------------------------------------------------------
// Inner class DynamicsBase
//------------------------------------------------------------------------------
//    void DynamicsBase::print(const string& s) const {
//        DynamicsBase::print(s);
//        cout << "   DynamicsBase covariance \n[" << covariance << "]" << endl;
//    }

//------------------------------------------------------------------------------
    bool DynamicsBase::equals(
            const DynamicsBase& other, double tol) const {
        return  equal_with_abs_tol(covariance, other.covariance, tol);
    }

//------------------------------------------------------------------------------
     void DynamicsBase::resetIntegration() {
        std::cout<<"GTSAM-resetDynamicsIntegration-开始4"<<std::endl;

        std::cout<<"GTSAM-sum_dt-开始"<<std::endl;
        sum_dt=0;
        vel0(0)=0;
        vel0(1)=0;
        vel0(2)=0;
        omiga0(0)=0;
        omiga0(1)=0;
        omiga0(2)=0;
        covariance.setZero();
        jacobian.setZero();
        noise.setZero();
        delta_p.setZero();
        delta_q.setIdentity();
        delta_v.setZero();
    }

//------------------------------------------------------------------------------
    void DynamicsBase::dynamics_Integration(double _dt,
                              const Eigen::Vector3d &_vel0, const Eigen::Vector3d &_omiga0,
                              const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                              Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                              bool update_jacobian)
    {
//        ROS_INFO("midpoint integration");
        Vector3d un_vel0 = delta_q * _vel0;
        result_delta_q = delta_q * Quaterniond(1, _omiga0(0) * _dt / 2, _omiga0(1) * _dt / 2, _omiga0(2) * _dt / 2);
        result_delta_p = delta_p + un_vel0 *_dt;

        if (update_jacobian)
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
            F.block<3, 3>(0, 0) = Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -delta_q.toRotationMatrix()*R_v_0_x *_dt;
            F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;

            MatrixXd V = MatrixXd::Zero(6, 6);
            V.block<3, 3>(0, 0) = delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3) * _dt;

            jacobian = F * jacobian;
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();
        }

    }

    Eigen::Matrix<double, 6, 1> DynamicsBase::evaluate(const Eigen::Vector3d Pi, const Eigen::Quaterniond Qi, const Eigen::Vector3d Pj, const Eigen::Quaterniond Qj)
    {
        Eigen::Matrix<double, 6, 1> residuals;
        residuals.setZero();
        residuals.block<3, 1>(0, 0) = Qi.inverse() * (Pj - Pi) - delta_p;
        residuals.block<3, 1>(3, 0) = 2 * (delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        //std::cout<<"residuals is "<<residuals<<std::endl;
        return residuals;
    }



// dynamicsFactor methods
//------------------------------------------------------------------------------
    dynamicsFactor::dynamicsFactor(Key pose_i, Key pose_j,
                         const DynamicsBase& db) :
            Base(noiseModel::Gaussian::Covariance(db.covariance), pose_i,pose_j), _DB_(db) {
    }

//------------------------------------------------------------------------------
    NonlinearFactor::shared_ptr dynamicsFactor::clone() const {
        return boost::static_pointer_cast<NonlinearFactor>(
                NonlinearFactor::shared_ptr(new This(*this)));
    }

//------------------------------------------------------------------------------
    std::ostream& operator<<(std::ostream& os, const dynamicsFactor& f) {
//        f._DB_.print("dynamicsFactor measurements:\n");
        os << "  noise model sigmas: " << f.noiseModel_->sigmas().transpose();//todo:noiseModel_是啥
        return os;
    }

//------------------------------------------------------------------------------
    void dynamicsFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
        cout << (s == "" ? s : s + "\n") << "dynamicsFactor("
             << "," << keyFormatter(this->key1()) << "," << keyFormatter(this->key2())
             << ")\n";
        cout << *this << endl;
    }

//------------------------------------------------------------------------------
    bool dynamicsFactor::equals(const NonlinearFactor& other, double tol) const {
        const This *e = dynamic_cast<const This*>(&other);
        const bool base = Base::equals(*e, tol);
        const bool db = _DB_.equals(e->_DB_, tol);
        return e != nullptr && base && db;
    }

//------------------------------------------------------------------------------
    Vector dynamicsFactor::evaluateError(const Pose3& pose_i,const Pose3& pose_j, boost::optional<Matrix&> H1,boost::optional<Matrix&> H2) const {
        //H1 对于posei的求雅克比的值，H2对于posej的求雅克比的值
        //返回vector
        Eigen::Vector3d Pi=pose_i.translation();
        Eigen::Quaterniond Qi(pose_i.rotation().toQuaternion());

        Eigen::Vector3d Pj=pose_j.translation();
        Eigen::Quaterniond Qj(pose_j.rotation().toQuaternion());


        Eigen::Matrix<double, 6, 1>  residual= dynamics_integration->evaluate(Pi, Qi, Pj, Qj);
        //Eigen::Matrix<double, 6, 6> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 6, 6>>(_DB_.covariance.inverse()).matrixL().transpose();
        //sqrt_info.setIdentity();
        //residual = sqrt_info * residual;
        if (dynamics_integration->jacobian.maxCoeff() > 1e8 || dynamics_integration->jacobian.minCoeff() < -1e8)
        {
            std::cout<<"numerical unstable in dynamics integration"<<std::endl;
            //std::cout << pre_integration->jacobian << std::endl;
            ///                ROS_BREAK();
        }

        if (H1)
        {
            Eigen::Matrix<double, 6, 7> jacobian_pose_i;
            jacobian_pose_i.setIdentity();

            jacobian_pose_i.block<3, 3>(0, 0) = -Qi.inverse().toRotationMatrix();
            jacobian_pose_i.block<3, 3>(0, 3) = Utility::skewSymmetric(Qi.inverse() * (Pj - Pi ));
            jacobian_pose_i.block<3, 3>(3, 3) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(dynamics_integration->delta_q)).bottomRightCorner<3, 3>();
            //cout<<"qleft:"<<Utility::Qleft(Qj.inverse() * Qi)<<endl;
            jacobian_pose_i = sqrt_info * jacobian_pose_i;
            *H1<<jacobian_pose_i;
            if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
            {
                std::cout<<"numerical unstable in preintegration"<<std::endl;
                //std::cout << sqrt_info << std::endl;
                return residual;
            }
            //cout<<"i:"<<jacobian_pose_i<<endl;
        }

        if (H2)
        {
            Eigen::Matrix<double, 6, 7> jacobian_pose_j;
            jacobian_pose_j.setIdentity();

            jacobian_pose_j.block<3, 3>(0, 0) = Qi.inverse().toRotationMatrix();
            jacobian_pose_j.block<3, 3>(3, 3) = Utility::Qleft(dynamics_integration->delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
            jacobian_pose_j = sqrt_info * jacobian_pose_j;
            *H2<<jacobian_pose_j;
        }

        return residual;//  xi << Rot3::Logmap(dR, (H1 || H2) ? &D_xi_R : 0), dt, dv;
    }


}