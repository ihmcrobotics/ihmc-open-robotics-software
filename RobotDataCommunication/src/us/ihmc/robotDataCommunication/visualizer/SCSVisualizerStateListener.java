package us.ihmc.robotDataCommunication.visualizer;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public interface SCSVisualizerStateListener
{
   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry);
}
