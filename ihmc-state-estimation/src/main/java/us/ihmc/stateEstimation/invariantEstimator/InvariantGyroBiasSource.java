package us.ihmc.stateEstimation.invariantEstimator;

/**
 * Where the invariant main estimator's gyro bias comes from.
 */
public enum InvariantGyroBiasSource
{
   /** The upstream IMU bias provider (the joint-level KF's base IMU bias, or zero), clamped. */
   PROVIDER,
   /** Re-measured at every two-foot standing rest and held in between ({@link RestGyroBiasEstimator}). */
   REST,
   /**
    * A filter state of the InEKF itself: roll/pitch observed through gravity and leg kinematics while walking, and
    * every rest estimate fused as a direct measurement (which also pins yaw).
    */
   STATE
}
