/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  PreintegrationParams.h
 *  @author Frank Dellaert
 **/

#pragma once

#include "PreintegratedRotation.h"
#include <boost/make_shared.hpp>

namespace gtsam {

/// Parameters for pre-integration:
/// Usage: Create just a single Params and pass a shared pointer to the constructor
struct GTSAM_EXPORT PreintegrationParams: PreintegratedRotationParams {
  Matrix3 wheelspeedsensorCovariance; ///< continuous-time "Covariance" of wheel speed sensor
  Matrix3 integrationCovariance; ///< continuous-time "Covariance" describing integration uncertainty

  /// Default constructor for serialization only
  PreintegrationParams()
      : PreintegratedRotationParams(),
        wheelspeedsensorCovariance(I_3x3),
        integrationCovariance(I_3x3){}

  /// The Params constructor insists on getting the navigation frame gravity vector
  /// For convenience, two commonly used conventions are provided by named constructors below
  PreintegrationParams(const Vector3& n_gravity)
      : PreintegratedRotationParams(),
        wheelspeedsensorCovariance(I_3x3),
        integrationCovariance(I_3x3) {}

  void print(const std::string& s="") const override;
  bool equals(const PreintegratedRotationParams& other, double tol) const override;

  void setWheelspeedsensorCovariance(const Matrix3& cov) { wheelspeedsensorCovariance = cov; }
  void setIntegrationCovariance(const Matrix3& cov)   { integrationCovariance = cov; }

  const Matrix3& getWheelspeedsensorCovariance() const { return wheelspeedsensorCovariance; }
  const Matrix3& getIntegrationCovariance()   const { return integrationCovariance; }


protected:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    namespace bs = ::boost::serialization;
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegratedRotationParams);
    ar & BOOST_SERIALIZATION_NVP(wheelspeedsensorCovariance);
    ar & BOOST_SERIALIZATION_NVP(integrationCovariance);
  }

#ifdef GTSAM_USE_QUATERNIONS
  // Align if we are using Quaternions
public:
	GTSAM_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

} // namespace gtsam
