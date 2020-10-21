package us.ihmc.avatar.ros.visualizer;

import us.ihmc.jMonkeyEngineToolkit.NullGraphics3DAdapter;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SCSROS2Visualizer
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Robot"),
                                                                               new NullGraphics3DAdapter(),
                                                                               new SimulationConstructionSetParameters());

   public SCSROS2Visualizer()
   {
      ROS2Debugger ros2Debugger = new ROS2Debugger();

      scs.addYoRegistry(registry);
      scs.setDT(1.0, 1);
      scs.skipLoadingDefaultConfiguration();
      scs.hideViewport();
   }

   public static void main(String[] args)
   {
      new SCSROS2Visualizer();
   }
}
