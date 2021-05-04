/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */


#include "../../include/ChaPreintegrationParams.h"

using namespace std;

namespace gtsam {

//------------------------------------------------------------------------------
void ChaPreintegrationParams::print(const string& s) const {
  ChaPreintegratedRotationParams::print(s);
  cout << "wheelspeedsensorCovariance:\n[\n" << wheelspeedsensorCovariance << "\n]"
       << endl;
  cout << "integrationCovariance:\n[\n" << integrationCovariance << "\n]"
       << endl;
  if (body_P_sensor) body_P_sensor->print("    ");

}

//------------------------------------------------------------------------------
bool ChaPreintegrationParams::equals(const ChaPreintegratedRotationParams& other,
                                  double tol) const {
  auto e = dynamic_cast<const ChaPreintegrationParams*>(&other);
  return e != nullptr && ChaPreintegratedRotationParams::equals(other, tol) &&
         equal_with_abs_tol(wheelspeedsensorCovariance, e->wheelspeedsensorCovariance,
                            tol) &&
         equal_with_abs_tol(integrationCovariance, e->integrationCovariance,
                            tol) ;
}

}  // namespace gtsam
