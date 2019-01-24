package us.ihmc.stateEstimation.humanoid;

import gnu.trove.map.TObjectDoubleMap;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.simulationconstructionset.util.RobotController;

/**
 * Provides a common interface for state estimator implementations.
 * <p>
 * Classes implementing this interface will extend the {@link RobotController} interface as well.
 * </p>
 */
public interface StateEstimatorController extends RobotController, StateEstimatorModeSubscriber
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
}
