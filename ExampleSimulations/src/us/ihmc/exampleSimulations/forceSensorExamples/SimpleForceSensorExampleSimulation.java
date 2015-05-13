package us.ihmc.exampleSimulations.forceSensorExamples;

import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class SimpleForceSensorExampleSimulation
{

   public static void main(String[] args)
   {
      SimpleForceSensorRobot robot = new SimpleForceSensorRobot();
      
      GroundContactModel groundContactModel = new LinearGroundContactModel(robot, robot.getRobotsYoVariableRegistry());
      robot.setGroundContactModel(groundContactModel);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 1);
      scs.startOnAThread();
   }
}
