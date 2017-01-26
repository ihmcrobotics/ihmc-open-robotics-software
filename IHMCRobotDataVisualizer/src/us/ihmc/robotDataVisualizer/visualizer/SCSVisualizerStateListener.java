package us.ihmc.robotDataVisualizer.visualizer;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public interface SCSVisualizerStateListener
{
   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry);
}
