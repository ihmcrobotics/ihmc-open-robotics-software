package us.ihmc.stateEstimation.invariantEstimator;

/**
 * How {@link InvariantEKFStateEstimator} models the error of the kinematic body-velocity measurement
 * ({@code invariantKinematicVelocityErrorModel}).
 * <p>
 * Against motion capture (Alex002, 2026-11-05) that error is about 3 cm/s per axis but stays correlated for 50-200 ms,
 * with a persistent offset of about 1 cm/s, while the measurement arrives every millisecond. Treated as white, one
 * error is counted hundreds of times: the innovation NIS is 0.3 (R looks too large) while the state NEES against
 * mocap is 13-22 (P far too small).
 * </p>
 */
public enum KinematicVelocityErrorModel
{
   /** White noise of std {@code invariantKinematicVelocityStd}, as before. */
   WHITE,
   /** A Gauss-Markov error state (τ, σ) estimated by the filter and subtracted from the measurement. */
   STATE,
   /** The same Gauss-Markov error carried in P as a Schmidt consider state: accounted for, never estimated. */
   CONSIDER;
}
