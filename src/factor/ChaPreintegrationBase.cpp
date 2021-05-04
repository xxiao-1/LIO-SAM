
#include "../../include/ChaPreintegrationBase.h"
#include <gtsam/base/numericalDerivative.h>
#include <boost/make_shared.hpp>

using namespace std;

namespace gtsam {

//------------------------------------------------------------------------------
ChaPreintegrationBase::ChaPreintegrationBase(const boost::shared_ptr<Params>& p,
                                       const Bias& biasHat)
    : p_(p), biasHat_(biasHat), deltaTij_(0.0) {
}

//------------------------------------------------------------------------------
ostream& operator<<(ostream& os, const ChaPreintegrationBase& pim) {
  os << "    deltaTij " << pim.deltaTij_ << endl;
  os << "    deltaRij.ypr = (" << pim.deltaRij().ypr().transpose() << ")" << endl;
  os << "    deltaPij " << Point3(pim.deltaPij()) << endl;
  os << "    gyrobias " << Point3(pim.biasHat_.gyroscope()) << endl;
  os << "    vel_bias " << Point3(pim.biasHat_.wheelspeed()) << endl;
  return os;
}

//------------------------------------------------------------------------------
void ChaPreintegrationBase::print(const string& s) const {
  cout << (s == "" ? s : s + "\n") << *this << endl;
}

//------------------------------------------------------------------------------
void ChaPreintegrationBase::resetIntegrationAndSetBias(const Bias& biasHat) {
	biasHat_ = biasHat;
	resetIntegration();
}

//------------------------------------------------------------------------------
pair<Vector3, Vector3> ChaPreintegrationBase::correctMeasurementsBySensorPose(
    const Vector3& unbiasedVel, const Vector3& unbiasedOmega,
    OptionalJacobian<3, 3> correctedVel_H_unbiasedVel,
    OptionalJacobian<3, 3> correctedVel_H_unbiasedOmega,
    OptionalJacobian<3, 3> correctedOmega_H_unbiasedOmega) const {
  assert(p().body_P_sensor);

  // Compensate for sensor-body displacement if needed: we express the quantities
  // (originally in the IMU frame) into the body frame
  // Equations below assume the "body" frame is the CG

  // Get sensor to body rotation matrix
  const Matrix3 bRs = p().body_P_sensor->rotation().matrix();

  // Convert angular velocity and acceleration from sensor to body frame
  Vector3 correctedVel = bRs * unbiasedVel;
  const Vector3 correctedOmega = bRs * unbiasedOmega;

  // Jacobians
  if (correctedVel_H_unbiasedVel) *correctedVel_H_unbiasedVel = bRs;
  if (correctedVel_H_unbiasedOmega) *correctedVel_H_unbiasedOmega = Z_3x3;
  if (correctedOmega_H_unbiasedOmega) *correctedOmega_H_unbiasedOmega = bRs;

  // Centrifugal acceleration
  const Vector3 b_arm = p().body_P_sensor->translation();
  if (!b_arm.isZero()) {
    // Subtract out the the centripetal acceleration from the unbiased one
    // to get linear acceleration vector in the body frame:
    const Matrix3 body_Omega_body = skewSymmetric(correctedOmega);
    const Vector3 b_velocity_bs = body_Omega_body * b_arm; // magnitude: omega * arm
    correctedVel -= body_Omega_body * b_velocity_bs;

    // Update derivative: centrifugal causes the correlation between acc and omega!!!
    if (correctedVel_H_unbiasedOmega) {
      double wdp = correctedOmega.dot(b_arm);
      const Matrix3 diag_wdp = Vector3::Constant(wdp).asDiagonal();
      *correctedVel_H_unbiasedOmega = -( diag_wdp
          + correctedOmega * b_arm.transpose()) * bRs.matrix()
          + 2 * b_arm * unbiasedOmega.transpose();
    }
  }

  return make_pair(correctedVel, correctedOmega);
}

//------------------------------------------------------------------------------
void ChaPreintegrationBase::integrateMeasurement(const Vector3& measuredVel,
    const Vector3& measuredOmega, double dt) {
  // NOTE(frank): integrateMeasurement always needs to compute the derivatives,
  // even when not of interest to the caller. Provide scratch space here.
  Matrix6 A;
  Matrix63 B, C;
  update(measuredVel, measuredOmega, dt, &A, &B, &C);
}

//------------------------------------------------------------------------------
ChaNavState ChaPreintegrationBase::predict(const ChaNavState& state_i,
    const chaBias::ConstantBias& bias_i, OptionalJacobian<6, 6> H1,
    OptionalJacobian<6, 6> H2) const {
  Matrix6 D_biasCorrected_bias;
  Vector6 biasCorrected = biasCorrectedDelta(bias_i,
                                             H2 ? &D_biasCorrected_bias : nullptr);

  // Correct for initial velocity and gravity
//  Matrix6 D_delta_state, D_delta_biasCorrected;
//  Vector6 xi = state_i.correctPIM(biasCorrected, deltaTij_, p().n_gravity,
//                                  p().omegaCoriolis, p().use2ndOrderCoriolis, H1 ? &D_delta_state : nullptr,
//                                  H2 ? &D_delta_biasCorrected : nullptr);
  Vector6 xi = biasCorrected;
  // Use retract to get back to ChaNavState manifold
  Matrix6 D_predict_state, D_predict_delta;
  ChaNavState state_j = state_i.retract(xi,
                                     H1 ? &D_predict_state : nullptr,
                                     H2 || H2 ? &D_predict_delta : nullptr);
  if (H1)
    *H1 = D_predict_state;
  if (H2)
    *H2 = D_predict_delta * D_biasCorrected_bias;
  return state_j;
}

//------------------------------------------------------------------------------
Vector6 ChaPreintegrationBase::computeError(const ChaNavState& state_i,
                                         const ChaNavState& state_j,
                                         const chaBias::ConstantBias& bias_i,
                                         OptionalJacobian<6, 6> H1,
                                         OptionalJacobian<6, 6> H2,
                                         OptionalJacobian<6, 6> H3) const {
  // Predict state at time j
  Matrix6 D_predict_state_i;
  Matrix6 D_predict_bias_i;
  ChaNavState predictedState_j = predict(
      state_i, bias_i, H1 ? &D_predict_state_i : 0, H3 ? &D_predict_bias_i : 0);

  // Calculate error
  Matrix6 D_error_state_j, D_error_predict;
  Vector6 error =
      state_j.localCoordinates(predictedState_j, H2 ? &D_error_state_j : 0,
                               H1 || H3 ? &D_error_predict : 0);

  if (H1) *H1 << D_error_predict* D_predict_state_i;
  if (H2) *H2 << D_error_state_j;
  if (H3) *H3 << D_error_predict* D_predict_bias_i;

  return error;
}

//------------------------------------------------------------------------------
Vector6 ChaPreintegrationBase::computeErrorAndJacobians(const Pose3& pose_i,
     const Pose3& pose_j, const chaBias::ConstantBias& bias_i, OptionalJacobian<6, 6> H1,
    OptionalJacobian<6, 6> H2, OptionalJacobian<6, 6> H3) const {

  // Note that derivative of constructors below is not identity for velocity, but
  // a 9*3 matrix == Z_3x3, Z_3x3, state.R().transpose()
  ChaNavState state_i(pose_i);
  ChaNavState state_j(pose_j);

  // Predict state at time j
  Matrix6 D_error_state_i, D_error_state_j;
  Vector6 error = computeError(state_i, state_j, bias_i,
                         H1 ? &D_error_state_i : 0, H2? &D_error_state_j : 0, H3);

  // Separate out derivatives in terms of 5 arguments
  // Note that doing so requires special treatment of velocities, as when treated as
  // separate variables the retract applied will not be the semi-direct product in ChaNavState
  // Instead, the velocities in nav are updated using a straight addition
  // This is difference is accounted for by the R().transpose calls below
  if (H1) *H1 << D_error_state_i.leftCols<6>();
  if (H2) *H2 << D_error_state_j.leftCols<6>();

  return error;
}

//------------------------------------------------------------------------------

}  // namespace gtsam
