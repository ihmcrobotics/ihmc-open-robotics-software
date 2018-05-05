package us.ihmc.robotics.robotController;

import us.ihmc.simulationConstructionSet.util.RobotControlElement;

public interface OutputProcessor extends RobotControlElement
{
   public abstract void update();
}
