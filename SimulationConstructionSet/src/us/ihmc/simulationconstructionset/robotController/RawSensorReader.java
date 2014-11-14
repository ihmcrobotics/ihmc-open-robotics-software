package us.ihmc.simulationconstructionset.robotController;

import us.ihmc.utilities.code.visualization.Visualize;

@Visualize
public interface RawSensorReader extends RobotControlElement
{
   public abstract void read();
}
