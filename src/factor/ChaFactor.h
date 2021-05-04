/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */
#pragma once

/* GTSAM includes */
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "ManifoldPreintegration.h"
#include <gtsam/base/debug.h>


namespace gtsam {
typedef ManifoldPreintegration PreintegrationType;

class GTSAM_EXPORT PreintegratedChaMeasurements: public PreintegrationType {

    friend class ChaFactor;

    friend class ChaFactor2;

    protected:

    Matrix6 preintMeasCov_; ///< COVARIANCE OF: [PreintROTATION PreintPOSITION ]
    ///< (first-order propagation from *measurementCovariance*).

    public:

    /// Default constructor for serialization and Cython wrapper
    PreintegratedChaMeasurements() {
        preintMeasCov_.setZero();
    }

    /**
      *  Constructor, initializes the class with no measurements
      *  @param p       Parameters, typically fixed in a single application
      *  @param biasHat Current estimate of acceleration and rotation rate biases
      */
    PreintegratedChaMeasurements(const boost::shared_ptr <PreintegrationParams> &p,
                                 const chaBias::ConstantBias &biasHat = chaBias::ConstantBias()) :
            PreintegrationType(p, biasHat) {
        preintMeasCov_.setZero();
    }

/**
  *  Construct preintegrated directly from members: base class and preintMeasCov
  *  @param base               PreintegrationType instance
  *  @param preintMeasCov      Covariance matrix used in noise model.
  */
    PreintegratedChaMeasurements(const PreintegrationType &base, const Matrix6 &preintMeasCov)
            : PreintegrationType(base),
              preintMeasCov_(preintMeasCov) {
    }

    /// Virtual destructor
    virtual ~

    PreintegratedChaMeasurements() {
    }

    /// print
    void print(const std::string &s = "Preintegrated Measurements:") const

    override;

    /// equals
    bool equals(const PreintegratedChaMeasurements &expected, double tol = 1e-9) const;

    /// Re-initialize PreintegratedChaMeasurements
    void resetIntegration()

    override;

    /**
     * Add a single IMU measurement to the preintegration.
     * @param measuredVel Measured vel (in body frame, as given by the sensor)
     * @param measuredOmega Measured angular velocity (as given by the sensor)
     * @param dt Time interval between this and the last IMU measurement
     */
    void integrateMeasurement(const Vector3 &measuredVel,
                              const Vector3 &measuredOmega, const double dt)

    override;

    /// Add multiple measurements, in matrix columns
    void integrateMeasurements(const Matrix &measuredVels, const Matrix &measuredOmegas,
                               const Matrix &dts);

    /// Return pre-integrated measurement covariance
    Matrix preintMeasCov() const { return preintMeasCov_; }

    private:

    /// Serialization function
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
        namespace bs = ::boost::serialization;
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationType);
        ar & BOOST_SERIALIZATION_NVP(preintMeasCov_);
    }
 };

/**
 * ChaFactor is a 5-ways factor involving previous state (pose and velocity of
 * the vehicle at previous time step), current state (pose and velocity at
 * current time step), and the bias estimate. Following the preintegration
 * scheme proposed in [2], the ChaFactor includes many IMU measurements, which
 * are "summarized" using the PreintegratedChaMeasurements class.
 * Note that this factor does not model "temporal consistency" of the biases
 * (which are usually slowly varying quantities), which is up to the caller.
 * See also CombinedChaFactor for a class that does this for you.
 *
 * @addtogroup SLAM
 */
class GTSAM_EXPORT ChaFactor: public NoiseModelFactor3<Pose3, Pose3, chaBias::ConstantBias> {
private:

  typedef ChaFactor This;
  typedef NoiseModelFactor3<Pose3, Pose3, chaBias::ConstantBias> Base;

  PreintegratedChaMeasurements _PIM_;

public:

  /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
  typedef typename boost::shared_ptr<ChaFactor> shared_ptr;
#else
  typedef boost::shared_ptr<ChaFactor> shared_ptr;
#endif

  /** Default constructor - only use for serialization */
  ChaFactor() {}

  /**
   * Constructor
   * @param pose_i Previous pose key
   * @param vel_i  Previous velocity key
   * @param pose_j Current pose key
   * @param vel_j  Current velocity key
   * @param bias   Previous bias key
   */
  ChaFactor(Key pose_i,  Key pose_j, Key bias,
      const PreintegratedChaMeasurements& preintegratedMeasurements);

  virtual ~ChaFactor() {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override;

  /// @name Testable
  /// @{
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const ChaFactor&);
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;
  /// @}

  /** Access the preintegrated measurements. */

  const PreintegratedChaMeasurements& preintegratedMeasurements() const {
    return _PIM_;
  }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const Pose3& pose_i, const Pose3& pose_j,
      const chaBias::ConstantBias& bias_i, boost::optional<Matrix&> H1 =
          boost::none, boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const override;

//#ifdef GTSAM_TANGENT_PREINTEGRATION
//  /// Merge two pre-integrated measurement classes
//  static PreintegratedChaMeasurements Merge(
//      const PreintegratedChaMeasurements& pim01,
//      const PreintegratedChaMeasurements& pim12);
//
//  /// Merge two factors
//  static shared_ptr Merge(const shared_ptr& f01, const shared_ptr& f12);
//#endif

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor5",
         boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(_PIM_);
  }
};
// class ChaFactor

/**
 * ChaFactor2 is a ternary factor that uses ChaNavStates rather than Pose/Velocity.
 * @addtogroup SLAM
 */
class GTSAM_EXPORT ChaFactor2 : public NoiseModelFactor3<ChaNavState, ChaNavState, chaBias::ConstantBias> {
private:

  typedef ChaFactor2 This;
  typedef NoiseModelFactor3<ChaNavState, ChaNavState, chaBias::ConstantBias> Base;

PreintegratedChaMeasurements _PIM_;

public:

  /** Default constructor - only use for serialization */
  ChaFactor2() {}

  /**
   * Constructor
   * @param state_i Previous state key
   * @param state_j Current state key
   * @param bias    Previous bias key
   */
  ChaFactor2(Key state_i, Key state_j, Key bias, const PreintegratedChaMeasurements& preintegratedMeasurements);

  virtual ~ChaFactor2() {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override;

  /// @name Testable
  /// @{
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const ChaFactor2&);
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;
  /// @}

  /** Access the preintegrated measurements. */

  const PreintegratedChaMeasurements& preintegratedMeasurements() const {
    return _PIM_;
  }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const ChaNavState& state_i, const ChaNavState& state_j,
                       const chaBias::ConstantBias& bias_i,  //
                       boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none,
                       boost::optional<Matrix&> H3 = boost::none) const override;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor3",
         boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(_PIM_);
  }
};
// class ChaFactor2

template <>
struct traits<PreintegratedChaMeasurements> : public Testable<PreintegratedChaMeasurements> {};

template <>
struct traits<ChaFactor> : public Testable<ChaFactor> {};

template <>
struct traits<ChaFactor2> : public Testable<ChaFactor2> {};

} /// namespace gtsam
