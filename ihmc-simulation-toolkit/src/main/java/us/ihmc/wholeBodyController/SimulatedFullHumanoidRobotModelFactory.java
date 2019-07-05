package us.ihmc.wholeBodyController;

import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;

public interface SimulatedFullHumanoidRobotModelFactory extends FullHumanoidRobotModelFactory
{
   HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping);

   default HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes)
   {
      return createHumanoidFloatingRootJointRobot(createCollisionMeshes, true);
   }
}
