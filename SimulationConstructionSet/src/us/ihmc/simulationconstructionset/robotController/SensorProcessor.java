package us.ihmc.simulationconstructionset.robotController;

import us.ihmc.utilities.code.visualization.Visualize;

@Visualize
public interface SensorProcessor extends RobotControlElement
{
   public abstract void update();
}
