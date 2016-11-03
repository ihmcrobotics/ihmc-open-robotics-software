package us.ihmc.wholeBodyController;

import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;

public interface SimulatedWholeBodyControllerParameters extends WholeBodyControllerParameters
{
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes);

}
