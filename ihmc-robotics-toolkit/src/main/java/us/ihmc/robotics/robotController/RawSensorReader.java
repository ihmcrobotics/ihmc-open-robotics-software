package us.ihmc.robotics.robotController;

import us.ihmc.simulationConstructionSet.util.RobotControlElement;

public interface RawSensorReader extends RobotControlElement
{
   public abstract void read();
}
