package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class LIPMWalkerSimulation
{
   public LIPMWalkerSimulation()
   {
      LIPMWalkerController controller = new LIPMWalkerController();
      LIPMWalkerRobot robotConstructor = new LIPMWalkerRobot();

      Robot robot = robotConstructor.getRobot();
      robot.setController(controller);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      scs.startOnAThread();
   }
   
   public static void main(String[] args)
   {
      LIPMWalkerSimulation simulation = new LIPMWalkerSimulation();
   }
}
