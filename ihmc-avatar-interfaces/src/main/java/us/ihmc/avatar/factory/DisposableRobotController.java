package us.ihmc.avatar.factory;

import us.ihmc.simulationconstructionset.util.RobotController;

public interface DisposableRobotController extends RobotController
{
   void dispose();
}
