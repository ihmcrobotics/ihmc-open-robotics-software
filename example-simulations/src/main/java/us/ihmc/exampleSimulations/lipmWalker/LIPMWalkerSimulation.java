package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class LIPMWalkerSimulation
{
   public LIPMWalkerSimulation()
   {
      LIPMWalkerController controller = new LIPMWalkerController();
      LIPMWalkerRobot robotConstructor = new LIPMWalkerRobot();

      Robot robot = robotConstructor.getRobot();

      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(robot, 14220.0, 150.6, 125.0, 300.0, robot.getRobotsYoRegistry());
      robot.setGroundContactModel(groundContactModel);

      robot.setController(controller);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      scs.startOnAThread();
   }
   
   public static void main(String[] args)
   {
      LIPMWalkerSimulation simulation = new LIPMWalkerSimulation();
   }
}
