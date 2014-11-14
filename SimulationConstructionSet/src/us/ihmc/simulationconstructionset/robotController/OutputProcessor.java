package us.ihmc.simulationconstructionset.robotController;

import us.ihmc.utilities.code.visualization.Visualize;

@Visualize
public interface OutputProcessor extends RobotControlElement
{
   public abstract void update();
}
