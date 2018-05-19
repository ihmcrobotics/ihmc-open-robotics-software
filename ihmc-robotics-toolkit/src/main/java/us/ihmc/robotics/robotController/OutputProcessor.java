package us.ihmc.robotics.robotController;

import us.ihmc.simulationconstructionset.util.RobotControlElement;

public interface OutputProcessor extends RobotControlElement
{
   public abstract void update();
}
