

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Manifold.h>

namespace gtsam {

/// Velocity is currently typedef'd to Vector3
typedef Vector3 Velocity3;

/**
 * Navigation state: Pose (rotation, translation) + velocity
 * NOTE(frank): it does not make sense to make this a Lie group, but it is a 9D manifold
 */
class GTSAM_EXPORT ChaNavState {
private:

  // TODO(frank):
  // - should we rename t_ to p_? if not, we should rename dP do dT
  Rot3 R_; ///< Rotation nRb, rotates points/velocities in body to points/velocities in nav
  Point3 t_; ///< position n_t, in nav frame

public:

  enum {
    dimension = 6
  };

//  typedef std::pair<Point3, Velocity3> PositionAndVelocity;

  /// @name Constructors
  /// @{

  /// Default constructor
  ChaNavState() :
      t_(0, 0, 0) {
  }
  /// Construct from attitude, position, velocity
  ChaNavState(const Rot3& R, const Point3& t) :
      R_(R), t_(t) {
  }
  /// Construct from pose and velocity
  ChaNavState(const Pose3& pose) :
      R_(pose.rotation()), t_(pose.translation()) {
  }
  /// Construct from SO(3) and R^6
  ChaNavState(const Matrix3& R, const Vector3& tv) :
      R_(R), t_(tv) {
  }
  /// Named constructor with derivatives
  static ChaNavState Create(const Rot3& R, const Point3& t,
      OptionalJacobian<6, 3> H1, OptionalJacobian<6, 3> H2);

  /// @}
  /// @name Component Access
  /// @{

  const Rot3& attitude(OptionalJacobian<3, 6> H = boost::none) const;
  const Point3& position(OptionalJacobian<3, 6> H = boost::none) const;
  const Pose3 pose() const {
    return Pose3(attitude(), position());
  }

  /// @}
  /// @name Derived quantities
  /// @{

  /// Return rotation matrix. Induces computation in quaternion mode
  Matrix3 R() const {
    return R_.matrix();
  }
  /// Return quaternion. Induces computation in matrix mode
  Quaternion quaternion() const {
    return R_.toQuaternion();
  }
  /// Return position as Vector3
  Vector3 t() const {
    return t_;
  }

  Vector3 bodyVelocity(const Vector3& vel, OptionalJacobian<3, 6> H= boost::none) const;
  /// Return matrix group representation, in MATLAB notation:
  /// nTb = [nRb 0 n_t; 0 nRb n_v; 0 0 1]
  /// With this embedding in GL(3), matrix product agrees with compose
//  Matrix7 matrix() const;

  /// @}
  /// @name Testable
  /// @{

  /// Output stream operator
  GTSAM_EXPORT
  friend std::ostream &operator<<(std::ostream &os, const ChaNavState& state);

  /// print
  void print(const std::string& s = "") const;

  /// equals
  bool equals(const ChaNavState& other, double tol = 1e-8) const;

  /// @}
  /// @name Manifold
  /// @{

  // Tangent space sugar.
  // TODO(frank): move to private navstate namespace in cpp
  static Eigen::Block<Vector6, 3, 1> dR(Vector6& v) {
    return v.segment<3>(0);
  }
  static Eigen::Block<Vector6, 3, 1> dP(Vector6& v) {
    return v.segment<3>(3);
  }
  static Eigen::Block<const Vector6, 3, 1> dR(const Vector6& v) {
    return v.segment<3>(0);
  }
  static Eigen::Block<const Vector6, 3, 1> dP(const Vector6& v) {
    return v.segment<3>(3);
  }

  /// retract with optional derivatives
  ChaNavState retract(const Vector6& v, //
      OptionalJacobian<6, 6> H1 = boost::none, OptionalJacobian<6, 6> H2 =
          boost::none) const;

  /// localCoordinates with optional derivatives
  Vector6 localCoordinates(const ChaNavState& g, //
      OptionalJacobian<6, 6> H1 = boost::none, OptionalJacobian<6, 6> H2 =
          boost::none) const;

  /// @}
  /// @name Dynamics
  /// @{

  /// Integrate forward in time given angular velocity and acceleration in body frame
  /// Uses second order integration for position, returns derivatives except dt.
  ChaNavState update(const Vector3& b_velocity, const Vector3& b_omega,
      const double dt, OptionalJacobian<6, 6> F, OptionalJacobian<6, 3> G1,
      OptionalJacobian<6, 3> G2) const;

  /// Compute tangent space contribution due to Coriolis forces
//  Vector6 coriolis(double dt, const Vector3& omega, bool secondOrder = false,
//      OptionalJacobian<6, 6> H = boost::none) const;

  /// Correct preintegrated tangent vector with our velocity and rotated gravity,
  /// taking into account Coriolis forces if omegaCoriolis is given.
//  Vector6 correctPIM(const Vector6& pim, double dt, const Vector3& n_gravity,
//      const boost::optional<Vector3>& omegaCoriolis, bool use2ndOrderCoriolis =
//          false, OptionalJacobian<6, 6> H1 = boost::none,
//      OptionalJacobian<6, 6> H2 = boost::none) const;

  /// @}

private:
  /// @{
  /// serialization
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(R_);
    ar & BOOST_SERIALIZATION_NVP(t_);
  }
  /// @}
};

// Specialize NavState traits to use a Retract/Local that agrees with IMUFactors
template<>
struct traits<ChaNavState> : internal::Manifold<ChaNavState> {
};

} // namespace gtsam
