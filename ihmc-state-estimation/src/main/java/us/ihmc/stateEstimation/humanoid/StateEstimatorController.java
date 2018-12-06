package us.ihmc.stateEstimation.humanoid;

import gnu.trove.map.TObjectDoubleMap;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.simulationconstructionset.util.RobotController;

/**
 * Provides a common interface for state estimator implementations.
 * <p>
 * Classes implementing this interface will extend the {@link RobotController} interface as well.
 * </p>
 */
public interface StateEstimatorController extends RobotController
{
   static final TObjectDoubleMap<String> EMPTY_JOINT_POSITION_MAP = new TObjectDoubleHashMap<>(0);

   /**
    * Initializes the root joint pose and joint positions in the estimator.
    *
    * @param rootJointTransform the transform of the floating root joint that is estimated.
    * @param jointPositions a map from joint names to initial joint positions.
    */
   public void initializeEstimator(RigidBodyTransform rootJointTransform, TObjectDoubleMap<String> jointPositions);

   /**
    * Initializes the root joint pose in the estimator.
    *
    * @param rootJointTransform the transform of the floating root joint that is estimated.
    */
   public default void initializeEstimator(RigidBodyTransform rootJointTransform)
   {
      initializeEstimator(rootJointTransform, EMPTY_JOINT_POSITION_MAP);
   }

   /**
    * Sets the operating mode of the state estimator. This will tell the estimator whether the robot
    * should be fixed in world or if the position of the robot should be estimated. If the robot is
    * hanging in the air and drifting away in normal operating mode this method can be used to fix
    * the robot and avoid the drift.
    * <p>
    * The implementation of this method is estimator specific. However, this method needs to be
    * thread-safe as this might be called from other threads then the one that is running the estimator.
    * </p>
    *
    * @param operatingMode to be set for the estimator.
    */
   public void requestStateEstimatorMode(StateEstimatorMode operatingMode);
}
