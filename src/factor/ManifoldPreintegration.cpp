/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include "ManifoldPreintegration.h"

using namespace std;

namespace gtsam {

//------------------------------------------------------------------------------
    ManifoldPreintegration::ManifoldPreintegration(
            const boost::shared_ptr<Params>& p, const Bias& biasHat) :
            PreintegrationBase(p, biasHat) {
        resetIntegration();
    }

//------------------------------------------------------------------------------
    void ManifoldPreintegration::resetIntegration() {
        deltaTij_ = 0.0;
        deltaXij_ = ChaNavState();
        delRdelBiasOmega_.setZero();
        delPdelBiasVel_.setZero();
        delPdelBiasOmega_.setZero();

    }

//------------------------------------------------------------------------------
    bool ManifoldPreintegration::equals(const ManifoldPreintegration& other,
                                        double tol) const {
        return p_->equals(*other.p_, tol) && std::abs(deltaTij_ - other.deltaTij_) < tol
               && biasHat_.equals(other.biasHat_, tol)
               && deltaXij_.equals(other.deltaXij_, tol)
               && equal_with_abs_tol(delRdelBiasOmega_, other.delRdelBiasOmega_, tol)
               && equal_with_abs_tol(delPdelBiasVel_, other.delPdelBiasVel_, tol)
               && equal_with_abs_tol(delPdelBiasOmega_, other.delPdelBiasOmega_, tol);
    }

//------------------------------------------------------------------------------
    void ManifoldPreintegration::update(const Vector3& measuredVel,
                                        const Vector3& measuredOmega, const double dt, Matrix6* A, Matrix63* B,
                                        Matrix63* C) {

        // Correct for bias in the sensor frame
        Vector3 vel = biasHat_.correctWheelSpeed(measuredVel);
        Vector3 omega = biasHat_.correctGyroscope(measuredOmega);

        // Possibly correct for sensor pose
        Matrix3 D_correctedVel_vel, D_correctedVel_omega, D_correctedOmega_omega;
        if (p().body_P_sensor)
            boost::tie(vel, omega) = correctMeasurementsBySensorPose(vel, omega,
                                                                     D_correctedVel_vel, D_correctedVel_omega, D_correctedOmega_omega);

        // Save current rotation for updating Jacobians
        const Rot3 oldRij = deltaXij_.attitude();

        // Do update
        deltaTij_ += dt;
        deltaXij_ = deltaXij_.update(vel, omega, dt, A, B, C); // functional

        if (p().body_P_sensor) {
            // More complicated derivatives in case of non-trivial sensor pose
            *C *= D_correctedOmega_omega;
            if (!p().body_P_sensor->translation().isZero())
                *C += *B * D_correctedVel_omega;
            *B *= D_correctedVel_vel; // NOTE(frank): needs to be last
        }

        // Update Jacobians
        // TODO(frank): Try same simplification as in new approach
        Matrix3 D_vel_R;
        oldRij.rotate(vel, D_vel_R);
        const Matrix3 D_vel_biasOmega = D_vel_R * delRdelBiasOmega_;

        const Vector3 integratedOmega = omega * dt;
        Matrix3 D_incrR_integratedOmega;
        const Rot3 incrR = Rot3::Expmap(integratedOmega, D_incrR_integratedOmega); // expensive !!
        const Matrix3 incrRt = incrR.transpose();
        delRdelBiasOmega_ = incrRt * delRdelBiasOmega_ - D_incrR_integratedOmega * dt;

        Matrix3d  R_vel;
        R_vel << 0, -vel(2), vel(1),
                vel(2), 0, -vel(0),
                -vel(1), vel(0), 0;
        double dt22 = 0.5 * dt * dt;
        const Matrix3 dRij = oldRij.matrix(); // expensive
        delPdelBiasVel_ += dRij * R_vel * dt ;
        delPdelBiasOmega_ +=  D_vel_biasOmega * dt
    }

//------------------------------------------------------------------------------
    Vector6 ManifoldPreintegration::biasCorrectedDelta(
            const chaBias::ConstantBias& bias_i, OptionalJacobian<6, 6> H) const {
        // Correct deltaRij, derivative is delRdelBiasOmega_
        const chaBias::ConstantBias biasIncr = bias_i - biasHat_;
        Matrix3 D_correctedRij_bias;
        const Vector3 biasInducedOmega = delRdelBiasOmega_ * biasIncr.gyroscope();
        const Rot3 correctedRij = deltaRij().expmap(biasInducedOmega, boost::none,
                                                    H ? &D_correctedRij_bias : 0);
        if (H)
            D_correctedRij_bias *= delRdelBiasOmega_;

        Vector6 xi;
        Matrix3 D_dR_correctedRij;
        // TODO(frank): could line below be simplified? It is equivalent to
        //   LogMap(deltaRij_.compose(Expmap(biasInducedOmega)))
        dR(xi) = Rot3::Logmap(correctedRij, H ? &D_dR_correctedRij : 0); //R
        dP(xi)= deltaPij() + delPdelBiasVel_ * biasIncr.wheelspeed() //P
                           + delPdelBiasOmega_ * biasIncr.gyroscope();

        if (H) {
            Matrix36 D_dR_bias, D_dP_bias;
            D_dR_bias << Z_3x3, D_dR_correctedRij * D_correctedRij_bias;
            D_dP_bias << delPdelBiasVel_, delPdelBiasOmega_;
            (*H) << D_dR_bias, D_dP_bias;
        }
        return xi;
    }

//------------------------------------------------------------------------------

}// namespace gtsam
