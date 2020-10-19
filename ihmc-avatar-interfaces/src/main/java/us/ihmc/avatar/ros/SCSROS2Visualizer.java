package us.ihmc.avatar.ros;

import us.ihmc.jMonkeyEngineToolkit.NullGraphics3DAdapter;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SCSROS2Visualizer
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final SimulationConstructionSet scs;

   public SCSROS2Visualizer()
   {
      scs = new SimulationConstructionSet(new Robot("Robot"), new NullGraphics3DAdapter(), new SimulationConstructionSetParameters());
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
