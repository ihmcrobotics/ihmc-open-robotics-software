package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;

public class BWCPlanarWalkerSimulation
{
   public BWCPlanarWalkerSimulation()
   {
      SimulationConstructionSet2 scs = new SimulationConstructionSet2();
//      scs.getGravity().setToZero();

      BWCPlanarWalkingRobotDefinition robotDefinition = new BWCPlanarWalkingRobotDefinition();
      Robot robot = new Robot(robotDefinition, scs.getInertialFrame());
      scs.addRobot(robot);
      scs.addTerrainObject(new SlopeGroundDefinition(0.0));

      // set up the controller robot that has convenience methods for us to do control things with.
      BWCPlanarWalkingRobot controllerRobot = new BWCPlanarWalkingRobot(robot);
      // create the robot controller
      BWCPlanarWalkingController controller = new BWCPlanarWalkingController(controllerRobot);
      // set the controller to control the robot.
      robot.addController(controller);

      scs.startSimulationThread();
      scs.simulate();
   }

   public static void main(String[] args)
   {
      new BWCPlanarWalkerSimulation();
   }
}
