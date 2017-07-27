package us.ihmc.wholeBodyController;

import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;

public interface SimulatedFullHumanoidRobotModelFactory extends FullHumanoidRobotModelFactory
{
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes);
}
