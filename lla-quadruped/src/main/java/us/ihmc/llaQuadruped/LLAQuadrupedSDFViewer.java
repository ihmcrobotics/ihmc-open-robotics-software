package us.ihmc.llaQuadruped;

import us.ihmc.euclid.tuple3D.Vector3D;

import us.ihmc.llaQuadrupedController.model.LLAQuadrupedModelFactory;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class LLAQuadrupedSDFViewer
{
   public static void main(String[] args)
   {
      LLAQuadrupedModelFactory modelFactory = new LLAQuadrupedModelFactory();
      FloatingRootJointRobot sdfRobot = new FloatingRootJointRobot(modelFactory.createSdfRobot());
      sdfRobot.setPositionInWorld(new Vector3D(0.0, 0.0, 0.6));
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
      scs.startOnAThread();
   }
}
