package us.ihmc.stateEstimation.humanoid;

/**
 * Which main state estimator {@link SwitchableMainStateEstimator} is currently letting drive the robot.
 *
 * <p>This is a <em>separate axis</em> from
 * {@link us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode}. That one selects
 * NORMAL vs FROZEN <em>within</em> whichever estimator is active ("hold the base"), and the controller
 * requests it on nearly every state change; folding the two together would swap estimators on every
 * stand-to-walk transition and would make "frozen InEKF" unrepresentable.</p>
 *
 * @author Lucas Libshutz
 */
public enum MainStateEstimatorType
{
   /** The kinematics-based DRC estimator, {@code DRCKinematicsBasedStateEstimator}. */
   DRC_KINEMATICS,
   /** The right-invariant EKF running as main, {@code InvariantMainStateEstimator}. */
   INVARIANT_EKF;

   public static final MainStateEstimatorType[] values = values();
}
