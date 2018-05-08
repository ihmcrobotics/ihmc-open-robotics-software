package us.ihmc.robotics.robotController;

import us.ihmc.simulationconstructionset.util.RobotControlElement;

public interface RawOutputWriter extends RobotControlElement
{
   public abstract void write();
}
