package us.ihmc.stateEstimation.humanoid;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.simulationconstructionset.util.RobotController;

/**
 * Provides a common interface for state estimator implementations.
 * <p>
 * Classes implementing this interface will extend the {@link RobotController} interface as well.
 * </p>
 */
public interface StateEstimatorController extends RobotController
{
   /**
    * Initializes the root joint pose in the estimator.
    *
    * @param rootJointTransform the transform of the floating root joint that is estimated.
    */
   public void initializeEstimator(RigidBodyTransform rootJointTransform);
}
