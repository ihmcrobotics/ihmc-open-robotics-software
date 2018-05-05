package us.ihmc.simulationConstructionSetTools.robotController;

import us.ihmc.simulationConstructionSet.util.RobotController;

public abstract class SliderBoardRobotController implements RobotController
{
   public abstract void initializeSliders();

   public abstract void register();
}
