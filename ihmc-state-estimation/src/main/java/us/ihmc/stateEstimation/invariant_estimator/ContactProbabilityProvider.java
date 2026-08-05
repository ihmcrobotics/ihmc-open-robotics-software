package us.ihmc.stateEstimation.invariant_estimator;

import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Per-foot contact-probability source for the invariant EKF's soft contact handling.
 *
 * <p>This is the seam between contact <em>sensing</em> and the filter's contact <em>trust</em>. Each tick
 * the estimator asks for a continuous probability {@code p ∈ [0, 1]} per foot and uses it to soften that
 * contact two ways (see {@link InvariantEKFStateEstimator}):
 * <ul>
 *   <li>the contact forward-kinematics measurement covariance {@code R_i} is inflated as {@code p → 0},
 *       so a swinging foot stops dragging the base velocity, while a stance foot keeps constraining it;</li>
 *   <li>the contact-position process noise {@code σ_{c,i}²} is inflated as {@code p → 0}, so a swing
 *       foot's world anchor "forgets" its stale position and re-anchors softly on touchdown without
 *       kicking the base.</li>
 * </ul>
 * </p>
 *
 * <p>The fallback implementation is {@link KinematicContactDetector} (forward-kinematics only, so it runs
 * on hardware with no foot force/torque sensing). This is intentionally the same role a learned
 * ContactNet covariance head will fill later — ContactNet either implements this interface directly or
 * rides the {@link ContactUpdater.LearnedContactCorrection} seam — so swapping the fallback for the
 * learned module is a one-line change at the estimator.</p>
 */
public interface ContactProbabilityProvider
{
   /**
    * Refreshes the per-foot contact probabilities for the current tick. The estimator calls this once per
    * {@code doControl}, after the shared reference frames have been updated and before the probabilities
    * are queried.
    */
   void update();

   /**
    * @param side the foot to query.
    * @return the contact probability {@code p ∈ [0, 1]} for {@code side} this tick: 1 = fully trusted
    *         contact, 0 = treat as not in contact (fully muted).
    */
   double getContactProbability(RobotSide side);

   /**
    * Discards whatever history this provider carries across ticks (finite-difference state, filters).
    *
    * <p>Call this after a <em>one-shot jump</em> of the estimated base pose — a gauge reset, not motion —
    * where a tick-to-tick difference taken across the jump is meaningless. Without it, a detector that
    * finite-differences sole position reads the jump as a velocity of {@code Δ/dt} and mistakes the reset
    * for a foot leaving the ground.</p>
    *
    * <p>The default is a no-op, which is correct for stateless and force-based providers (e.g.
    * {@link FootSwitchContactProbabilityProvider}): they read the sensors afresh each tick and never see
    * the estimated base pose at all.</p>
    */
   default void reset()
   {
   }
}
