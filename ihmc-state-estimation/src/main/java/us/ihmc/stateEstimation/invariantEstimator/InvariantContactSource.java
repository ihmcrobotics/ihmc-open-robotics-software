package us.ihmc.stateEstimation.invariantEstimator;

/**
 * Selects where the {@link InvariantMainStateEstimator}'s per-foot contact probability comes from when the
 * invariant InEKF runs as the main floating-base estimator.
 *
 * <p>The choice is made at the estimator-thread factory level (see
 * {@code AvatarEstimatorThreadFactory#setInvariantContactSource}), which is shared by both the simulation
 * and hardware avatar stacks — so the same contact source runs unchanged in sim and on the robot.</p>
 */
public enum InvariantContactSource
{
   /**
    * The robot's production foot switches — on Alex the joint-torque based {@code JointTorqueBasedFootSwitch}
    * selected by {@code StateEstimatorParameters#getFootSwitchFactories()}, the same contact detection the
    * DRC estimator and the walking controller trust. They are built on the estimator's own model and sensor
    * stream (force-sensor holder + joint torques), exactly as the DRC estimator builds them, so this source
    * behaves identically in simulation and on hardware. Independent of the invariant filter's own base
    * estimate, so it is not circular when the filter is the main estimator. This is the default.
    */
   FOOT_SWITCHES,
   /**
    * The invariant estimator's built-in forward-kinematics height detector
    * ({@link KinematicContactDetector}). It reads sole heights in world through the estimator model, which
    * the invariant filter itself poses when promoted to main — circular in that mode. Keep only as an A/B
    * baseline for isolating contact detection as an error source; it needs no foot force/torque sensing.
    */
   KINEMATIC_DETECTOR
}
