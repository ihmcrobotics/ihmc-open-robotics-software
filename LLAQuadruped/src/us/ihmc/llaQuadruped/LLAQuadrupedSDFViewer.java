package us.ihmc.llaQuadruped;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class LLAQuadrupedSDFViewer
{
   public static void main(String[] args)
   {
      LLAQuadrupedModelFactory modelFactory = new LLAQuadrupedModelFactory();
      SDFRobot sdfRobot = modelFactory.createSdfRobot();
      sdfRobot.setPositionInWorld(new Vector3d(0.0, 0.0, 0.6));
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
      scs.startOnAThread();
   }
}
