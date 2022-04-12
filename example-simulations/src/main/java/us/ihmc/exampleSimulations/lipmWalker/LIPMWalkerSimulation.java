package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.exampleSimulations.lipmWalker.nonPlanar.NonPlanarLIPMWalkerControllerBhavyansh;
import us.ihmc.exampleSimulations.lipmWalker.nonPlanar.NonPlanarLIPMWalkerRobot;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class LIPMWalkerSimulation
{
   public LIPMWalkerSimulation()
   {

      NonPlanarLIPMWalkerRobot nonPlanarRobotConstructor = new NonPlanarLIPMWalkerRobot();
      Robot robot = nonPlanarRobotConstructor.getRobot();
      NonPlanarLIPMWalkerControllerBhavyansh controller = new NonPlanarLIPMWalkerControllerBhavyansh(nonPlanarRobotConstructor);

//      LIPMWalkerRobot planarRobotConstructor = new LIPMWalkerRobot();
//      Robot robot = planarRobotConstructor.getRobot();
//      LIPMHoppingControllerBhavyansh controller = new LIPMHoppingControllerBhavyansh(planarRobotConstructor);
//      LIPMWalkerControllerBhavyansh controller = new LIPMWalkerControllerBhavyansh(planarRobotConstructor);
//      LIPMWalkerControllerTobi controller = new LIPMWalkerControllerTobi(planarRobotConstructor);
//      LIPMWalkerController controller = new LIPMWalkerController(planarRobotConstructor);

      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(robot, 14220.0, 150.6, 125.0, 100.0, robot.getRobotsYoRegistry());
      robot.setGroundContactModel(groundContactModel);

      robot.setController(controller);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setSimulateNoFasterThanRealTime(true);
      scs.setCameraTracking(true, true, true, false);

      scs.startOnAThread();
   }
   
   public static void main(String[] args)
   {
      LIPMWalkerSimulation simulation = new LIPMWalkerSimulation();
   }
}
