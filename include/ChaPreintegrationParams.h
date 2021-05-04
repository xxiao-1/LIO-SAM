

#pragma once

#include "ChaPreintegratedRotation.h"
#include <boost/make_shared.hpp>

namespace gtsam {

/// Parameters for pre-integration:
/// Usage: Create just a single Params and pass a shared pointer to the constructor
struct GTSAM_EXPORT ChaPreintegrationParams: ChaPreintegratedRotationParams {
  Matrix3 wheelspeedsensorCovariance; ///< continuous-time "Covariance" of wheel speed sensor
  Matrix3 integrationCovariance; ///< continuous-time "Covariance" describing integration uncertainty

  /// Default constructor for serialization only
  ChaPreintegrationParams()
      : ChaPreintegratedRotationParams(),
        wheelspeedsensorCovariance(I_3x3),
        integrationCovariance(I_3x3){}
    // Default Params for a Z-down navigation frame, such as NED: gravity points along positive Z-axis
    static boost::shared_ptr<ChaPreintegrationParams> MakeSharedU() {
        return boost::shared_ptr<ChaPreintegrationParams>(new ChaPreintegrationParams());
    }
  void print(const std::string& s="") const override;
  bool equals(const ChaPreintegratedRotationParams& other, double tol) const override;

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
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(ChaPreintegratedRotationParams);
    ar & BOOST_SERIALIZATION_NVP(wheelspeedsensorCovariance);
    ar & BOOST_SERIALIZATION_NVP(integrationCovariance);
  }

#ifdef GTSAM_USE_QUATERNIONS
  // Align if we are using Quaternions
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

} // namespace gtsam
