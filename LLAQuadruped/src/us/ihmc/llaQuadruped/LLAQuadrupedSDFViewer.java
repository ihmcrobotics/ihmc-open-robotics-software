package us.ihmc.llaQuadruped;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class LLAQuadrupedSDFViewer
{
   public static void main(String[] args)
   {
      LLAQuadrupedModelFactory modelFactory = new LLAQuadrupedModelFactory();
      SDFRobot sdfRobot = modelFactory.createSdfRobot();
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
      scs.startOnAThread();
   }
}
