package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class LIPMWalkerSimulation
{
   private static final double simDT = 0.0004;
   private static final double controlDT = simDT;

   public LIPMWalkerSimulation()
   {
      LIPMWalkerRobot robotConstructor = new LIPMWalkerRobot();

      Robot robot = robotConstructor.getRobot();
//      LIPMWalkerControllerBhavyansh controller = new LIPMWalkerControllerBhavyansh(robotConstructor);
//      LIPMWalkerControllerTobi controller = new LIPMWalkerControllerTobi(robotConstructor);
//      LIPMWalkerController controller = new LIPMWalkerController(robotConstructor);
//      LIPMWalkerControllerGMN controller = new LIPMWalkerControllerGMN(robotConstructor);
      LIPMWalkerControllerRG controller = new LIPMWalkerControllerRG(robotConstructor, controlDT);

      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(robot, 14220.0, 150.6, 125.0, 100.0, robot.getRobotsYoRegistry());
      robot.setGroundContactModel(groundContactModel);

      robot.setController(controller, (int) (controlDT / simDT));
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(50000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setDT(simDT, (int) (controlDT / simDT));
      scs.setSimulateNoFasterThanRealTime(true);

      scs.startOnAThread();
   }
   
   public static void main(String[] args)
   {
      LIPMWalkerSimulation simulation = new LIPMWalkerSimulation();
   }
}
