

#pragma once

#include "ChaNavState.h"
#include "PreintegrationBase.h"

namespace gtsam {

/**
 * IMU pre-integration on NavSatet manifold.
 * This corresponds to the original RSS paper (with one difference: V is rotated)
 */
    class GTSAM_EXPORT ManifoldPreintegration : public PreintegrationBase {
    protected:

    /**
     * Pre-integrated navigation state, from frame i to frame j
     * Note: relative position does not take into account velocity at time i, see deltap+, in [2]
     * Note: velocity is now also in frame i, as opposed to deltaVij in [2]
     */
    ChaNavState deltaXij_;
    Matrix3 delRdelBiasOmega_; ///< Jacobian of preintegrated rotation w.r.t. angular rate bias
    Matrix3 delPdelBiasVel_;   ///< Jacobian of preintegrated position w.r.t. wheel spped sensor bias
    Matrix3 delPdelBiasOmega_; ///< Jacobian of preintegrated position w.r.t. angular rate bias

    /// Default constructor for serialization
    ManifoldPreintegration() {
        resetIntegration();
    }

    public:
    /// @name Constructors
    /// @{

    /**
     *  Constructor, initializes the variables in the base class
     *  @param p    Parameters, typically fixed in a single application
     *  @param bias Current estimate of acceleration and rotation rate biases
     */
    ManifoldPreintegration(const boost::shared_ptr<Params>& p,
                           const chaBias::ConstantBias& biasHat = chaBias::ConstantBias());

    /// @}

    /// @name Basic utilities
    /// @{
    /// Re-initialize PreintegratedMeasurements
    void resetIntegration() override;

    /// @}

    /// @name Instance variables access
    /// @{
ChaNavState deltaXij() const override { return deltaXij_; }
Rot3     deltaRij() const override { return deltaXij_.attitude(); }
Vector3  deltaPij() const override { return deltaXij_.position(); }

Matrix3  delRdelBiasOmega() const { return delRdelBiasOmega_; }
Matrix3  delPdelbiasVel() const { return delPdelBiasVel_; }
Matrix3  delPdelBiasOmega() const { return delPdelBiasOmega_; }

/// @name Testable
/// @{
bool equals(const ManifoldPreintegration& other, double tol) const;
/// @}

/// @name Main functionality
/// @{

/// Update preintegrated measurements and get derivatives
/// It takes measured quantities in the j frame
/// Modifies preintegrated quantities in place after correcting for bias and possibly sensor pose
/// NOTE(frank): implementation is different in two versions
void update(const Vector3& measuredVel, const Vector3& measuredOmega, const double dt,
            Matrix6* A, Matrix63* B, Matrix63* C) override;

/// Given the estimate of the bias, return a NavState tangent vector
/// summarizing the preintegrated IMU measurements so far
/// NOTE(frank): implementation is different in two versions
Vector6 biasCorrectedDelta(const chaBias::ConstantBias& bias_i,
                           OptionalJacobian<6, 6> H = boost::none) const override;

/** Dummy clone for MATLAB */
virtual boost::shared_ptr<ManifoldPreintegration> clone() const {
    return boost::shared_ptr<ManifoldPreintegration>();
}

/// @}

private:
/** Serialization function */
friend class boost::serialization::access;
template<class ARCHIVE>
void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    namespace bs = ::boost::serialization;
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationBase);
    ar & BOOST_SERIALIZATION_NVP(deltaXij_);
    ar & BOOST_SERIALIZATION_NVP(delRdelBiasOmega_);
    ar & BOOST_SERIALIZATION_NVP(delPdelBiasVel_);
    ar & BOOST_SERIALIZATION_NVP(delPdelBiasOmega_);
}
};

} /// namespace gtsam
