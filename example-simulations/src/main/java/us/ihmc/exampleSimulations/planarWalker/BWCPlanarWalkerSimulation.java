package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.robot.Robot;

public class BWCPlanarWalkerSimulation
{
   public BWCPlanarWalkerSimulation()
   {
      int simTicksPerControlTick = 3;

      SimulationConstructionSet2 scs = new SimulationConstructionSet2();
      scs.setBufferRecordTickPeriod(simTicksPerControlTick);
      //      scs.getGravity().setToZero();

      BWCPlanarWalkingRobotDefinition robotDefinition = new BWCPlanarWalkingRobotDefinition();
      Robot robot = new Robot(robotDefinition, scs.getInertialFrame());
      scs.addRobot(robot);
      scs.addTerrainObject(new BWCSlopeGroundDefinition(0.0));

      // set up the controller robot that has convenience methods for us to do control things with.
      BWCPlanarWalkingRobot controllerRobot = new BWCPlanarWalkingRobot(robot, scs.getTime());
      // create the robot controller
      BWCPlanarWalkingController controller = new BWCPlanarWalkingController(controllerRobot, RobotSide.LEFT);
      // set the controller to control the robot.
      robot.addThrottledController(controller, scs.getDT() * simTicksPerControlTick);

      scs.startSimulationThread();
      scs.simulate();
   }

   public static void main(String[] args)
   {
      new BWCPlanarWalkerSimulation();
   }
}