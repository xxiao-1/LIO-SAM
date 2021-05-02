/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */


#include "ChaFactor.h"

/* External or standard includes */
#include <ostream>

namespace gtsam {

using namespace std;

//------------------------------------------------------------------------------
// Inner class PreintegratedChaMeasurements
//------------------------------------------------------------------------------
void PreintegratedChaMeasurements::print(const string& s) const {
  PreintegrationType::print(s);
  cout << "    preintMeasCov \n[" << preintMeasCov_ << "]" << endl;
}

//------------------------------------------------------------------------------
bool PreintegratedChaMeasurements::equals(
    const PreintegratedChaMeasurements& other, double tol) const {
  return PreintegrationType::equals(other, tol)
      && equal_with_abs_tol(preintMeasCov_, other.preintMeasCov_, tol);
}

//------------------------------------------------------------------------------
void PreintegratedChaMeasurements::resetIntegration() {
  PreintegrationType::resetIntegration();
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void PreintegratedChaMeasurements::integrateMeasurement(
    const Vector3& measureVel, const Vector3& measuredOmega, double dt) {
  if (dt <= 0) {
    throw std::runtime_error(
        "PreintegratedChaMeasurements::integrateMeasurement: dt <=0");
  }

  // Update preintegrated measurements (also get Jacobian)
  Matrix6 A;  // overall Jacobian wrt preintegrated measurements (df/dx)
  Matrix63 B, C;
  PreintegrationType::update(measureVel, measuredOmega, dt, &A, &B, &C);

  // first order covariance propagation:
  // as in [2] we consider a first order propagation that can be seen as a
  // prediction phase in EKF

  // propagate uncertainty
  // TODO(frank): use noiseModel routine so we can have arbitrary noise models.
  const Matrix3& vCov = p().wheelspeedsensorCovariance;
  const Matrix3& wCov = p().gyroscopeCovariance;
  const Matrix3& iCov = p().integrationCovariance;

  // (1/dt) allows to pass from continuous time noise to discrete time noise
  preintMeasCov_ = A * preintMeasCov_ * A.transpose();
  preintMeasCov_.noalias() += B * (vCov / dt) * B.transpose();
  preintMeasCov_.noalias() += C * (wCov / dt) * C.transpose();

  // NOTE(frank): (Gi*dt)*(C/dt)*(Gi'*dt), with Gi << Z_3x3, I_3x3, Z_3x3
  preintMeasCov_.block<3, 3>(3, 3).noalias() += iCov * dt;
}

//------------------------------------------------------------------------------
void PreintegratedChaMeasurements::integrateMeasurements(
    const Matrix& measureVels, const Matrix& measuredOmegas,
    const Matrix& dts) {
  assert(
      measureVels.rows() == 3 && measuredOmegas.rows() == 3 && dts.rows() == 1);
  assert(dts.cols() >= 1);
  assert(measureVels.cols() == dts.cols());
  assert(measuredOmegas.cols() == dts.cols());
  size_t n = static_cast<size_t>(dts.cols());
  for (size_t j = 0; j < n; j++) {
    integrateMeasurement(measureVels.col(j), measuredOmegas.col(j), dts(0, j));
  }
}


//------------------------------------------------------------------------------
// ChaFactor methods
//------------------------------------------------------------------------------
ChaFactor::ChaFactor(Key pose_i, Key pose_j, Key bias,
    const PreintegratedChaMeasurements& pim) :
    Base(noiseModel::Gaussian::Covariance(pim.preintMeasCov_), pose_i,
        pose_j, bias ), _PIM_(pim) {
}

//------------------------------------------------------------------------------
NonlinearFactor::shared_ptr ChaFactor::clone() const {
  return boost::static_pointer_cast<NonlinearFactor>(
      NonlinearFactor::shared_ptr(new This(*this)));
}

//------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const ChaFactor& f) {
  f._PIM_.print("preintegrated measurements:\n");
  os << "  noise model sigmas: " << f.noiseModel_->sigmas().transpose();
  return os;
}

//------------------------------------------------------------------------------
void ChaFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << (s == "" ? s : s + "\n") << "ChaFactor(" << keyFormatter(this->key1())
       << "," << keyFormatter(this->key2()) << "," << keyFormatter(this->key3())
       << ")\n";
  cout << *this << endl;
}

//------------------------------------------------------------------------------
bool ChaFactor::equals(const NonlinearFactor& other, double tol) const {
  const This *e = dynamic_cast<const This*>(&other);
  const bool base = Base::equals(*e, tol);
  const bool pim = _PIM_.equals(e->_PIM_, tol);
  return e != nullptr && base && pim;
}

//------------------------------------------------------------------------------
Vector ChaFactor::evaluateError(const Pose3& pose_i,
    const Pose3& pose_j,  const chaBias::ConstantBias& bias_i, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2, boost::optional<Matrix&> H3) const {
  return _PIM_.computeErrorAndJacobians(pose_i,  pose_j,  bias_i, H1, H2, H3);
}


//------------------------------------------------------------------------------
// ChaFactor2 methods
//------------------------------------------------------------------------------
ChaFactor2::ChaFactor2(Key state_i, Key state_j, Key bias,
    const PreintegratedChaMeasurements& pim) :
    Base(noiseModel::Gaussian::Covariance(pim.preintMeasCov_), state_i, state_j,
        bias), _PIM_(pim) {
}

//------------------------------------------------------------------------------
NonlinearFactor::shared_ptr ChaFactor2::clone() const {
  return boost::static_pointer_cast<NonlinearFactor>(
      NonlinearFactor::shared_ptr(new This(*this)));
}

//------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const ChaFactor2& f) {
  f._PIM_.print("preintegrated measurements:\n");
  os << "  noise model sigmas: " << f.noiseModel_->sigmas().transpose();
  return os;
}

//------------------------------------------------------------------------------
void ChaFactor2::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << (s == "" ? s : s + "\n") << "ChaFactor2("
       << keyFormatter(this->key1()) << "," << keyFormatter(this->key2()) << ","
       << keyFormatter(this->key3()) << ")\n";
  cout << *this << endl;
}

//------------------------------------------------------------------------------
bool ChaFactor2::equals(const NonlinearFactor& other, double tol) const {
  const This *e = dynamic_cast<const This*>(&other);
  const bool base = Base::equals(*e, tol);
  const bool pim = _PIM_.equals(e->_PIM_, tol);
  return e != nullptr && base && pim;
}

//------------------------------------------------------------------------------
Vector ChaFactor2::evaluateError(const NavState& state_i,
    const NavState& state_j,
    const chaBias::ConstantBias& bias_i, //
    boost::optional<Matrix&> H1, boost::optional<Matrix&> H2,
    boost::optional<Matrix&> H3) const {
  return _PIM_.computeError(state_i, state_j, bias_i, H1, H2, H3);
}

//------------------------------------------------------------------------------

}
// namespace gtsam
