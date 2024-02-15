package us.ihmc.exampleSimulations.planarWalker.BWC;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.robot.Robot;

public class BWCPlanarWalkingSimulation
{

   public BWCPlanarWalkingSimulation ()
   {
      int simTicksPerControlTick = 3;

      SimulationConstructionSet2 scs = new SimulationConstructionSet2();
      scs.setBufferRecordTickPeriod(simTicksPerControlTick);

      scs.getGravity().set(0.0, 0.0, -9.8);

      BWCPlanarWalkingRobotDefinition robotDefinition = new BWCPlanarWalkingRobotDefinition();
      Robot robot = new Robot(robotDefinition, scs.getInertialFrame());
      scs.addRobot(robot);
      scs.addTerrainObject(new FlatGroundDefinition());

      BWCPlanarWalkingRobot controllerRobot = new BWCPlanarWalkingRobot(robot, scs.getTime());
      BWCPlanarWalkingController controller = new BWCPlanarWalkingController(controllerRobot, RobotSide.LEFT);

      robot.addThrottledController(controller, scs.getDT() * simTicksPerControlTick);

      scs.startSimulationThread();
      scs.setShutdownSessionOnVisualizerClose(true);
   }

   public static void main(String[] args)
   {
      new BWCPlanarWalkingSimulation();
   }
}
