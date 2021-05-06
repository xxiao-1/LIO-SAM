/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */


#include "../../include/ChaNavState.h"

using namespace std;

namespace gtsam {

#define TIE(R,t,x) const Rot3& R = (x).R_;const Point3& t = (x).t_;

//------------------------------------------------------------------------------
ChaNavState ChaNavState::Create(const Rot3& R, const Point3& t,
    OptionalJacobian<6, 3> H1, OptionalJacobian<6, 3> H2) {
  if (H1)
    *H1 << I_3x3, Z_3x3;
  if (H2)
    *H2 << Z_3x3, R.transpose();
  return ChaNavState(R, t);
}

//------------------------------------------------------------------------------
const Rot3& ChaNavState::attitude(OptionalJacobian<3, 6> H) const {
  if (H)
    *H << I_3x3, Z_3x3;
  return R_;
}

//------------------------------------------------------------------------------
const Point3& ChaNavState::position(OptionalJacobian<3, 6> H) const {
  if (H)
    *H << Z_3x3, R();
  return t_;
}

//------------------------------------------------------------------------------
Vector3 ChaNavState::bodyVelocity(const Vector3& vel, OptionalJacobian<3, 6> H) const {
  const Rot3& nRb = R_;
  const Vector3& n_v = vel;
  Matrix3 D_bv_nRb;
  Vector3 b_v = nRb.unrotate(n_v, H ? &D_bv_nRb : 0);
  if (H)
    *H << D_bv_nRb, Z_3x3;
  return b_v;
}

//------------------------------------------------------------------------------
//Matrix7 ChaNavState::matrix() const {
//  Matrix3 R = this->R();
//  Matrix7 T;
//  T << R, Z_3x3, t(), Z_3x3, R, v(), Vector6::Zero().transpose(), 1.0;
//  return T;
//}

//------------------------------------------------------------------------------
ostream& operator<<(ostream& os, const ChaNavState& state) {
  os << "R: " << state.attitude() << "\n";
  os << "p: " << state.position().transpose() << "\n";
  return os;
}

//------------------------------------------------------------------------------
void ChaNavState::print(const string& s) const {
  cout << (s.empty() ? s : s + " ") << *this << endl;
}

//------------------------------------------------------------------------------
bool ChaNavState::equals(const ChaNavState& other, double tol) const {
  return R_.equals(other.R_, tol) && traits<Point3>::Equals(t_, other.t_, tol);
}

//------------------------------------------------------------------------------
ChaNavState ChaNavState::retract(const Vector6& xi, //
    OptionalJacobian<6, 6> H1, OptionalJacobian<6, 6> H2) const {
  TIE(nRb, n_t, *this);
  Matrix3 D_bRc_xi, D_R_nRb, D_t_nRb;
  const Rot3 bRc = Rot3::Expmap(dR(xi), H2 ? &D_bRc_xi : 0);
  const Rot3 nRc = nRb.compose(bRc, H1 ? &D_R_nRb : 0);
  const Point3 t = n_t + nRb.rotate(dP(xi), H1 ? &D_t_nRb : 0);
  if (H1) {
    *H1 << D_R_nRb, Z_3x3, //
    // Note(frank): the derivative of n_t with respect to xi is nRb
    // We pre-multiply with nRc' to account for NavState::Create
    // Then we make use of the identity nRc' * nRb = bRc'
    nRc.transpose() * D_t_nRb, bRc.transpose();
  }
  if (H2) {
    *H2 << D_bRc_xi, Z_3x3, //
    Z_3x3, bRc.transpose(); //
  }
  return ChaNavState(nRc, t);
}

//------------------------------------------------------------------------------
Vector6 ChaNavState::localCoordinates(const ChaNavState& g, //
    OptionalJacobian<6, 6> H1, OptionalJacobian<6, 6> H2) const {
  Matrix3 D_dR_R, D_dt_R, D_dv_R;
  const Rot3 dR = R_.between(g.R_, H1 ? &D_dR_R : 0);
  const Point3 dt = R_.unrotate(g.t_ - t_, H1 ? &D_dt_R : 0);

  Vector6 xi;
  Matrix3 D_xi_R;
  xi << Rot3::Logmap(dR, (H1 || H2) ? &D_xi_R : 0), dt;
  if (H1) {
    *H1 << D_xi_R * D_dR_R, Z_3x3,//
    D_dt_R, -I_3x3;
  }
  if (H2) {
    *H2 << D_xi_R, Z_3x3,//
    Z_3x3, dR.matrix();
  }
  return xi;
}

//------------------------------------------------------------------------------
// sugar for derivative blocks
#define D_R_R(H) (H)->block<3,3>(0,0)
#define D_R_t(H) (H)->block<3,3>(0,3)
#define D_t_R(H) (H)->block<3,3>(3,0)
#define D_t_t(H) (H)->block<3,3>(3,3)

//------------------------------------------------------------------------------
ChaNavState ChaNavState::update(const Vector3& b_velocity, const Vector3& b_omega,
    const double dt, OptionalJacobian<6, 6> F, OptionalJacobian<6, 3> G1,
    OptionalJacobian<6, 3> G2) const {

  Vector6 xi;
  Matrix36 D_xiP_state;
  Vector3 b_v = ChaNavState::bodyVelocity( b_velocity,F ? &D_xiP_state : 0);
//  double dt22 = 0.5 * dt * dt;
  // Integrate on tangent space. TODO(frank): coriolis?
  dR(xi) << dt * b_omega;
  dP(xi) << dt * b_v;

  // Bring back to manifold
  Matrix6 D_newState_xi;
  ChaNavState newState = retract(xi, F, G1 || G2 ? &D_newState_xi : 0);

  // Derivative wrt state is computed by retract directly
  // However, as dP(xi) also depends on state, we need to add that contribution
  if (F) {
    F->middleRows<3>(3) += dt * D_t_t(F) * D_xiP_state; //TODO whether to change this line
  }
  // derivative wrt velocity
  if (G1) {
    // D_newState_dPxi = D_newState_xi.middleCols<3>(3)
    // D_dPxi_acc = dt22 * I_3x3
    // D_newState_dVxi = D_newState_xi.rightCols<3>()
    // D_dVxi_acc = dt * I_3x3
    // *G2 = D_newState_acc = D_newState_dPxi * D_dPxi_acc + D_newState_dVxi * D_dVxi_acc
    *G1 = D_newState_xi.rightCols<3>() * dt;
  }
  // derivative wrt omega
  if (G2) {
    // D_newState_dRxi = D_newState_xi.leftCols<3>()
    // D_dRxi_omega = dt * I_3x3
    // *G1 = D_newState_omega = D_newState_dRxi * D_dRxi_omega
    *G2 = D_newState_xi.leftCols<3>() * dt;
  }
  return newState;
}

//------------------------------------------------------------------------------no need if 不需要矫正重力向量
//Vector6 ChaNavState::coriolis(double dt, const Vector3& omega, bool secondOrder,
//    OptionalJacobian<6, 6> H) const {
//  TIE(nRb, n_t, *this);
//  const double dt2 = dt * dt;
//  const Vector3 omega_cross_vel = omega.cross(n_v);
//
//  // Get perturbations in nav frame
//  Vector6 n_xi, xi;
//  Matrix3 D_dR_R, D_dP_R, D_dV_R, D_body_nav;
//  dR(n_xi) << ((-dt) * omega);
//  dP(n_xi) << ((-dt2) * omega_cross_vel); // NOTE(luca): we got rid of the 2 wrt INS paper
//  dV(n_xi) << ((-2.0 * dt) * omega_cross_vel);
//  if (secondOrder) {
//    const Vector3 omega_cross2_t = omega.cross(omega.cross(n_t));
//    dP(n_xi) -= (0.5 * dt2) * omega_cross2_t;
//    dV(n_xi) -= dt * omega_cross2_t;
//  }
//
//  // Transform n_xi into the body frame
//  xi << nRb.unrotate(dR(n_xi), H ? &D_dR_R : 0, H ? &D_body_nav : 0),
//        nRb.unrotate(dP(n_xi), H ? &D_dP_R : 0),
//        nRb.unrotate(dV(n_xi), H ? &D_dV_R : 0);
//
//  if (H) {
//    H->setZero();
//    const Matrix3 Omega = skewSymmetric(omega);
//    const Matrix3 D_cross_state = Omega * R();
//    H->setZero();
//    D_R_R(H) << D_dR_R;
//    D_t_v(H) << D_body_nav * (-dt2) * D_cross_state;
//    D_t_R(H) << D_dP_R;
//    D_v_v(H) << D_body_nav * (-2.0 * dt) * D_cross_state;
//    D_v_R(H) << D_dV_R;
//    if (secondOrder) {
//      const Matrix3 D_cross2_state = Omega * D_cross_state;
//      D_t_t(H) -= D_body_nav * (0.5 * dt2) * D_cross2_state;
//      D_v_t(H) -= D_body_nav * dt * D_cross2_state;
//    }
//  }
//  return xi;
//}

//------------------------------------------------------------------------------得到重力向量后的修正no need
//Vector6 NavState::correctPIM(const Vector6& pim, double dt,
//    const Vector3& n_gravity, const boost::optional<Vector3>& omegaCoriolis,
//    bool use2ndOrderCoriolis, OptionalJacobian<6, 6> H1,
//    OptionalJacobian<6, 6> H2) const {
//  const Rot3& nRb = R_;
//  const Velocity3& n_v = v_; // derivative is Ri !
//  const double dt22 = 0.5 * dt * dt;
//
//  Vector6 xi;
//  Matrix3 D_dP_Ri1, D_dP_Ri2, D_dP_nv, D_dV_Ri;
//  dR(xi) = dR(pim);
//  dP(xi) = dP(pim)
//      + dt * nRb.unrotate(n_v, H1 ? &D_dP_Ri1 : 0, H2 ? &D_dP_nv : 0)
//      + dt22 * nRb.unrotate(n_gravity, H1 ? &D_dP_Ri2 : 0);
//  dV(xi) = dV(pim) + dt * nRb.unrotate(n_gravity, H1 ? &D_dV_Ri : 0);
//
//  if (omegaCoriolis) {
//    xi += coriolis(dt, *omegaCoriolis, use2ndOrderCoriolis, H1);
//  }
//
//  if (H1 || H2) {
//    Matrix3 Ri = nRb.matrix();
//
//    if (H1) {
//      if (!omegaCoriolis)
//        H1->setZero(); // if coriolis H1 is already initialized
//      D_t_R(H1) += dt * D_dP_Ri1 + dt22 * D_dP_Ri2;
//      D_t_v(H1) += dt * D_dP_nv * Ri;
//      D_v_R(H1) += dt * D_dV_Ri;
//    }
//    if (H2) {
//      H2->setIdentity();
//    }
//  }
//
//  return xi;
//}
//------------------------------------------------------------------------------

}/// namespace gtsam
