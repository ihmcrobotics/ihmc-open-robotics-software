package us.ihmc.stateEstimation.humanoid;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.simulationconstructionset.util.RobotController;

public interface DRCStateEstimatorInterface extends RobotController
{
   public void initializeEstimator(RigidBodyTransform rootJointTransform);
}
