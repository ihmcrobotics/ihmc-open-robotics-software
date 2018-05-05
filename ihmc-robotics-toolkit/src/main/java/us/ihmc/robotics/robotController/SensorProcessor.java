package us.ihmc.robotics.robotController;

import us.ihmc.simulationConstructionSet.util.RobotControlElement;

public interface SensorProcessor extends RobotControlElement
{
   public abstract void update();
}
