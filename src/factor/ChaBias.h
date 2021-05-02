/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ImuBias.h
 * @date  Feb 2, 2012
 * @author Vadim Indelman, Stephen Williams
 */

#pragma once

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/VectorSpace.h>
#include <iosfwd>
#include <boost/serialization/nvp.hpp>

namespace gtsam {

/// All bias models live in the imuBias namespace
    namespace chaBias {

        class GTSAM_EXPORT ConstantBias{
                private:
                Vector3 biasVel_;
                Vector3 biasGyro_;

                public:
                /// dimension of the variable - used to autodetect sizes
                static const size_t dimension = 6;

                ConstantBias() :
                biasVel_(0.0, 0.0, 0.0), biasGyro_(0.0, 0.0, 0.0) {
                }

                ConstantBias(const Vector3& biasVel, const Vector3& biasGyro) :
                biasVel_(biasVel), biasGyro_(biasGyro) {
                }

                explicit ConstantBias(const Vector6& v) :
                biasVel_(v.head<3>()), biasGyro_(v.tail<3>()) {
                }

                /** return the velocity and gyro biases in a single vector */
                Vector6 vector() const {
                    Vector6 v;
                    v << biasVel_, biasGyro_;
                    return v;
                }

                /** get wheelspeed bias */
                const Vector3& wheelspeed() const {
                    return biasVel_;
                }

                /** get gyroscope bias */
                const Vector3& gyroscope() const {
                    return biasGyro_;
                }

                /** Correct an wheel speed sensor measurement using this bias model, and optionally compute Jacobians */
                Vector3 correctWheelSpeed(const Vector3& measurement,
                OptionalJacobian<3, 6> H1 = boost::none,
                OptionalJacobian<3, 3> H2 = boost::none) const {
                    Matrix3 r_v;
                    r_v << measurement(0), 0, 0,
                            0, measurement(1), 0,
                            0, 0, measurement(2);
                    if (H1) (*H1) << R_w_x, Z_3x3;
                    if (H2) (*H2) << 1+biasVel_(0), 0, 0,
                                     0, 1+biasVel_(1), 0,
                                     0, 0, 1+biasVel_(2);
                    Vector3 tmp=measurement.array()*biasVel_.array();
                    return measurement + tmp ;
                }

                /** Correct a gyroscope measurement using this bias model, and optionally compute Jacobians */
                Vector3 correctGyroscope(const Vector3& measurement,
                OptionalJacobian<3, 6> H1 = boost::none,
                OptionalJacobian<3, 3> H2 = boost::none) const {
                    if (H1) (*H1) << Z_3x3, -I_3x3;
                    if (H2) (*H2) << I_3x3;
                    return measurement - biasGyro_;
                }

                /// @}
                /// @name Testable
                /// @{

                /// ostream operator
                GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                const ConstantBias& bias);

                /// print with optional string
                void print(const std::string& s = "") const;

                /** equality up to tolerance */
                inline bool equals(const ConstantBias& expected, double tol = 1e-5) const {
                    return equal_with_abs_tol(biasVel_, expected.biasVel_, tol)
                           && equal_with_abs_tol(biasGyro_, expected.biasGyro_, tol);
                }

                /// @}
                /// @name Group
                /// @{

                /** identity for group operation */
                static ConstantBias identity() {
                    return ConstantBias();
                }

                /** inverse */
                inline ConstantBias operator-() const {
                    return ConstantBias(-biasVel_, -biasGyro_);
                }

                /** addition of vector on right */
                ConstantBias operator+(const Vector6& v) const {
                    return ConstantBias(biasVel_ + v.head<3>(), biasGyro_ + v.tail<3>());
                }

                /** addition */
                ConstantBias operator+(const ConstantBias& b) const {
                    return ConstantBias(biasVel_ + b.biasVel_, biasGyro_ + b.biasGyro_);
                }

                /** subtraction */
                ConstantBias operator-(const ConstantBias& b) const {
                    return ConstantBias(biasVel_ - b.biasVel_, biasGyro_ - b.biasGyro_);
                }

                /// @}

                /// @name Deprecated
                /// @{
                ConstantBias inverse() {
                    return -(*this);
                }
                ConstantBias compose(const ConstantBias& q) {
                    return (*this) + q;
                }
                ConstantBias between(const ConstantBias& q) {
                    return q - (*this);
                }
                Vector6 localCoordinates(const ConstantBias& q) {
                    return between(q).vector();
                }
                ConstantBias retract(const Vector6& v) {
                    return compose(ConstantBias(v));
                }
                static Vector6 Logmap(const ConstantBias& p) {
                    return p.vector();
                }
                static ConstantBias Expmap(const Vector6& v) {
                    return ConstantBias(v);
                }
                /// @}

                private:

                /// @name Advanced Interface
                /// @{

                /** Serialization function */
                friend class boost::serialization::access;
                template<class ARCHIVE>
                void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
                    ar &BOOST_SERIALIZATION_NVP(biasVel_);
                    ar &BOOST_SERIALIZATION_NVP(biasGyro_);
                }


                public:
                GTSAM_MAKE_ALIGNED_OPERATOR_NEW
                /// @}

        }; // ConstantBias class
    } // namespace imuBias

    template<>
    struct traits<chaBias::ConstantBias> : public internal::VectorSpace<
            chaBias::ConstantBias> {
    };

} // namespace gtsam

