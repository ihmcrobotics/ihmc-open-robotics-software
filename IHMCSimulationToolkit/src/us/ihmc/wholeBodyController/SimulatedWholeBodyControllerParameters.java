package us.ihmc.wholeBodyController;

import us.ihmc.humanoidRobotics.HumanoidFloatingRootJointRobot;

public interface SimulatedWholeBodyControllerParameters extends WholeBodyControllerParameters
{
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes);

}
