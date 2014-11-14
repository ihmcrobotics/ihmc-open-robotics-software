package us.ihmc.simulationconstructionset.robotController;

import us.ihmc.utilities.code.visualization.Visualize;

@Visualize
public interface RawOutputWriter extends RobotControlElement
{
   public abstract void write();
}
