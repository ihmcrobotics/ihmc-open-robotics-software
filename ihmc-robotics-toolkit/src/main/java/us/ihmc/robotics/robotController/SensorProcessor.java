package us.ihmc.robotics.robotController;

import us.ihmc.simulationconstructionset.util.RobotControlElement;

public interface SensorProcessor extends RobotControlElement
{
   public abstract void update();
}
