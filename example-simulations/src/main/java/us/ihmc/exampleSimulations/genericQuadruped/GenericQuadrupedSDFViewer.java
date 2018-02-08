package us.ihmc.exampleSimulations.genericQuadruped;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedModelFactory;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class GenericQuadrupedSDFViewer
{
   public static void main(String[] args)
   {
      GenericQuadrupedModelFactory modelFactory = new GenericQuadrupedModelFactory();
      FloatingRootJointRobot sdfRobot = new FloatingRootJointRobot(modelFactory.createSdfRobot());
      sdfRobot.setPositionInWorld(new Vector3D(0.0, 0.0, 0.6));
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
      scs.startOnAThread();
   }
}
