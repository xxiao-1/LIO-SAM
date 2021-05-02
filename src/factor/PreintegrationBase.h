/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */



#pragma once

#include "PreintegrationParams.h"
#include "ChaNavState.h"
#include "ChaBias.h"
#include <gtsam/linear/NoiseModel.h>

#include <iosfwd>
#include <string>
#include <utility>

namespace gtsam {

/**
 * PreintegrationBase is the base class for PreintegratedMeasurements
 * (in ImuFactor) and CombinedPreintegratedMeasurements (in CombinedImuFactor).
 * It includes the definitions of the preintegrated variables and the methods
 * to access, print, and compare them.
 */
class GTSAM_EXPORT PreintegrationBase {
 public:
  typedef chaBias::ConstantBias Bias;
  typedef PreintegrationParams Params;

 protected:
  boost::shared_ptr<Params> p_;

  /// Acceleration and gyro bias used for preintegration
  Bias biasHat_;

  /// Time interval from i to j
  double deltaTij_;

  /// Default constructor for serialization
  PreintegrationBase() {}

  /// Virtual destructor for serialization
  virtual ~PreintegrationBase() {}

 public:
  /// @name Constructors
  /// @{

  /**
   *  Constructor, initializes the variables in the base class
   *  @param p    Parameters, typically fixed in a single application
   *  @param bias Current estimate of acceleration and rotation rate biases
   */
  PreintegrationBase(const boost::shared_ptr<Params>& p,
      const chaBias::ConstantBias& biasHat = chaBias::ConstantBias());

  /// @}

  /// @name Basic utilities
  /// @{
  /// Re-initialize PreintegratedMeasurements
  virtual void resetIntegration() = 0;

  /// @name Basic utilities
  /// @{
  /// Re-initialize PreintegratedMeasurements and set new bias
  void resetIntegrationAndSetBias(const Bias& biasHat);

  /// check parameters equality: checks whether shared pointer points to same Params object.
  bool matchesParamsWith(const PreintegrationBase& other) const {
    return p_.get() == other.p_.get();
  }

  /// shared pointer to params
  const boost::shared_ptr<Params>& params() const {
    return p_;
  }

  /// const reference to params
  Params& p() const {
    return *p_;
  }

  /// @}

  /// @name Instance variables access
  /// @{
  const chaBias::ConstantBias& biasHat() const { return biasHat_; }
  double deltaTij() const { return deltaTij_; }

  virtual Vector3  deltaPij() const = 0;
  virtual Rot3     deltaRij() const = 0;
  virtual ChaNavState deltaXij() const = 0;

  // Exposed for MATLAB
  Vector6 biasHatVector() const { return biasHat_.vector(); }
  /// @}

  /// @name Testable
  /// @{
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const PreintegrationBase& pim);
  virtual void print(const std::string& s="") const;
  /// @}

  /// @name Main functionality
  /// @{

  /**
   * Subtract estimate and correct for sensor pose
   * Compute the derivatives due to non-identity body_P_sensor (rotation and centrifugal acc)
   * Ignore D_correctedOmega_measuredAcc as it is trivially zero
   */
  std::pair<Vector3, Vector3> correctMeasurementsBySensorPose(
      const Vector3& unbiasedAcc, const Vector3& unbiasedOmega,
      OptionalJacobian<3, 3> correctedAcc_H_unbiasedAcc = boost::none,
      OptionalJacobian<3, 3> correctedAcc_H_unbiasedOmega = boost::none,
      OptionalJacobian<3, 3> correctedOmega_H_unbiasedOmega = boost::none) const;

  /**
   *  Update preintegrated measurements and get derivatives
   * It takes measured quantities in the j frame
   * Modifies preintegrated quantities in place after correcting for bias and possibly sensor pose
   */
  virtual void update(const Vector3& measuredAcc, const Vector3& measuredOmega,
      const double dt, Matrix9* A, Matrix93* B, Matrix93* C) = 0;

  /// Version without derivatives
  virtual void integrateMeasurement(const Vector3& measuredAcc,
      const Vector3& measuredOmega, const double dt);

  /// Given the estimate of the bias, return a ChaNavState tangent vector
  /// summarizing the preintegrated IMU measurements so far
  virtual Vector6 biasCorrectedDelta(const chaBias::ConstantBias& bias_i,
      OptionalJacobian<6, 6> H = boost::none) const = 0;

  /// Predict state at time j
  ChaNavState predict(const ChaNavState& state_i, const chaBias::ConstantBias& bias_i, const Vector3 vel_i,
                   OptionalJacobian<6, 6> H1 = boost::none,
                   OptionalJacobian<6, 6> H2 = boost::none) const;

  /// Calculate error given navStates
  Vector9 computeError(const ChaNavState& state_i, const ChaNavState& state_j,
                       const chaBias::ConstantBias& bias_i, const Vector3 vel_i,
                       OptionalJacobian<9, 9> H1, OptionalJacobian<9, 9> H2,
                       OptionalJacobian<9, 6> H3) const;

  /**
   * Compute errors w.r.t. preintegrated measurements and jacobians
   * wrt pose_i, vel_i, bias_i, pose_j, bias_j
   */
  Vector9 computeErrorAndJacobians(const Pose3& pose_i, const Pose3& pose_j,
      const chaBias::ConstantBias& bias_i, const Vector3 vel_i, OptionalJacobian<6, 6> H1 =
      OptionalJacobian<6, 6> H2 = boost::none, OptionalJacobian<6, 6> H3 =
          boost::none) const;

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(p_);
    ar & BOOST_SERIALIZATION_NVP(biasHat_);
    ar & BOOST_SERIALIZATION_NVP(deltaTij_);
  }

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

}  /// namespace gtsam
