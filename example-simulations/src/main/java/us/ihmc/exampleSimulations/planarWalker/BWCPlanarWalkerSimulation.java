package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;

public class BWCPlanarWalkerSimulation
{
  public BWCPlanarWalkerSimulation()
  {
    try
    {
      int simTicksPerControlTick = 3;
      ContactPointBasedContactParameters contactParameters = ContactPointBasedContactParameters.defaultParameters();
      PhysicsEngineFactory physicsEngineFactory =
          PhysicsEngineFactory.newContactPointBasedPhysicsEngineFactory(contactParameters);
      SimulationConstructionSet2 scs = new SimulationConstructionSet2("bloop", physicsEngineFactory);
      scs.setBufferRecordTickPeriod(simTicksPerControlTick);

      BWCPlanarWalkingRobotDefinition robotDefinition = new BWCPlanarWalkingRobotDefinition();
      Robot robot = new Robot(robotDefinition, scs.getInertialFrame());
      scs.addRobot(robot);
      scs.addTerrainObject(new SlopeGroundDefinition(0.0));

      BWCPlanarWalkingRobot controllerRobot = new BWCPlanarWalkingRobot(robot, scs.getTime());
      if (controllerRobot.getYoRegistry() == null)
      {
        throw new RuntimeException("Registry setup failed: YoRegistry is null.");
      }

      scs.addYoGraphic(controllerRobot.getSCS2YoGraphics());

      BWCPlanarWalkingController controller = new BWCPlanarWalkingController(controllerRobot, RobotSide.LEFT);
      scs.addYoGraphic(controller.getSCS2YoGraphics());
      robot.addThrottledController(controller, scs.getDT() * simTicksPerControlTick);

      scs.startSimulationThread();
      scs.simulate();
      System.out.println("Simulation setup complete. Running...");
    }
    catch (Exception e)
    {
      System.err.println("Error occurred during simulation setup or execution:");
      e.printStackTrace();
    }
  }

  //    public BWCPlanarWalkerSimulation()
  //   {
  //      int simTicksPerControlTick = 3;
  //
  //
  //      ContactPointBasedContactParameters contactParameters = ContactPointBasedContactParameters.defaultParameters();
  //
  //      PhysicsEngineFactory physicsEngineFactory =
  //      PhysicsEngineFactory.newContactPointBasedPhysicsEngineFactory(contactParameters);
  ////      PhysicsEngineFactory physicsEngineFactory = PhysicsEngineFactory.newContactPointBasedPhysicsEngineFactory();
  //      SimulationConstructionSet2 scs = new SimulationConstructionSet2("bloop", physicsEngineFactory);
  //      scs.setBufferRecordTickPeriod(simTicksPerControlTick);
  ////      scs.getGravity().setToZero();
  //
  //      BWCPlanarWalkingRobotDefinition robotDefinition = new BWCPlanarWalkingRobotDefinition();
  //      Robot robot = new Robot(robotDefinition, scs.getInertialFrame());
  //      scs.addRobot(robot);
  //      scs.addTerrainObject(new SlopeGroundDefinition(0.0));
  //
  //      // set up the controller robot that has convenience methods for us to do control things with.
  //      BWCPlanarWalkingRobot controllerRobot = new BWCPlanarWalkingRobot(robot, scs.getTime());
  //
  //      // create the robot controller
  //      BWCPlanarWalkingController controller = new BWCPlanarWalkingController(controllerRobot, RobotSide.LEFT);
  //
  //      scs.addYoGraphic(controllerRobot.getSCS2YoGraphics());
  //      scs.addYoGraphic(controller.getSCS2YoGraphics());
  //
  //      // set the controller to control the robot.
  //      robot.addThrottledController(controller, scs.getDT() * simTicksPerControlTick);
  //
  //
  //      PushRobotControllerSCS2 pushRobotController = new PushRobotControllerSCS2(scs.getTime(),
  //                                                                                robot,
  //                                                                                controllerRobot.getFloatingJoint().getName(),
  //                                                                                new Vector3D());
  //      pushRobotController.addPushButtonToSCS(scs);
  //      pushRobotController.setPushDelay(0.0);
  //      pushRobotController.setPushForceDirection(new Vector3D(1.0, 0.0, 0.0));
  //      pushRobotController.setPushForceMagnitude(50.00);
  //      pushRobotController.setPushDuration(0.2);
  //
  //      scs.addRegistry(pushRobotController.getYoRegistry());
  //      scs.addYoGraphic(pushRobotController.getForceVizDefinition());
  //
  //
  //      scs.startSimulationThread();
  //      scs.simulate();
  //   }

  public static void main(String[] args)
  {
    try
    {
      System.out.println("Starting BWCPlanarWalkerSimulation...");
      new BWCPlanarWalkerSimulation();
      System.out.println("Simulation setup complete. Running...");
    }
    catch (Exception e)
    {
      System.err.println("Error occurred during simulation setup or execution:");
      e.printStackTrace();
    }
  }
}
