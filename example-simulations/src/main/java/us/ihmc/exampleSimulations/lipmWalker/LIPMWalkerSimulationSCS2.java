package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.robotics.physics.PhysicsEngineTools;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.session.tools.SCS1GraphicConversionTools;
import us.ihmc.scs2.simulation.parameters.ContactParametersReadOnly;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.robot.Robot;

public class LIPMWalkerSimulationSCS2
{
   private static final double simDT = 0.0004;
   private static final double controlDT = simDT;

   public LIPMWalkerSimulationSCS2()
   {
      //      LIPMWalkerRobot robotConstructor = new LIPMWalkerRobot();
      //      Robot robot = robotConstructor.getRobot();
      //      LIPMWalkerControllerJae controller = new LIPMWalkerControllerJae(robotConstructor, controlDT);
      ////      LIPMWalkerControllerRG controller = new LIPMWalkerControllerRG(robotConstructor, controlDT);
      //
      //      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(robot, 14220.0, 150.6, 125.0, 100.0, robot.getRobotsYoRegistry());
      //      robot.setGroundContactModel(groundContactModel);
      //      robot.setController(controller, (int) (controlDT / simDT));

      // Create an instance of the robot arm
      //      RobotDefinition walkerDef = new LIPMWalkerRobotDefinition2().createRobotDefinition();
      LIPMWalkerRobotDefinition walkerDef = new LIPMWalkerRobotDefinition();

      // Define ground contact parameters
      ContactPointBasedContactParameters contact = ContactPointBasedContactParameters.defaultParameters();
      //      contact.setKxy(1000.0);
      //      contact.setBxy(500.0);
      //      contact.setKz(650.0);
      //      contact.setBz(500.0);
      contact.setKxy(40000.0);
      contact.setBxy(100.0);
      contact.setKz(500.0);
      contact.setBz(250.0);

      // Instantiate a SCS object
      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.contactPointBasedPhysicsEngineFactory(contact));
      //      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.impulseBasedPhysicsEngineFactory((ContactParametersReadOnly) contact));

      // The gravity has to be explicitly defined for the controller core (maybe a robot on the Moon someday...?)
      double gravityMagnitude = 9.81;

      // Make sure the simulation and the controller are using the same value for the gravity.
      scs.getGravity().set(0.0, 0.0, -gravityMagnitude);

      // This time, we will make the controller run at a slower frequency than the simulation.
      double controllerDT = 1.0e-2;

      // The simulation time step.
      double simulateDT = 4.0e-4;
      scs.setDT(simulateDT);
      // Set the frequency at which data is logged.
      scs.setBufferRecordTickPeriod(10);
      scs.changeBufferSize(100000);
      scs.setRealTimeRateSimulation(true);

      // Generate a robot for the simulation
      Robot walker = scs.addRobot(walkerDef);

      // Add a terrain
      // flat ground
      //      GeometryDefinition groundGeometryDefinition = new Box3DDefinition(10.0, 10.0, 0.1);
      //      addVisualDefinition(new VisualDefinition(new RigidBodyTransform(), groundGeometryDefinition, new MaterialDefinition(ColorDefinitions.DarkGray())));
      //      addCollisionShapeDefinition(new CollisionShapeDefinition(new RigidBodyTransform(), groundGeometryDefinition));
      scs.addTerrainObject(new GroundDefinition());

      ControllerJae controller = new ControllerJae(walker.getControllerInput(),
                                                   walker.getControllerOutput(),
                                                   controllerDT,
                                                   gravityMagnitude,
                                                   walkerDef);
      controller.initialize();
      scs.addYoEntry("measuredCapturePointX");
      scs.addYoEntry("measuredCapturePointY");
      scs.addYoEntry("measuredCapturePointZ");
      scs.addYoGraphic(controller.getYoGraphicDefinition());

      walker.addController(controller);

      // Camera settings
      scs.setCameraFocusPosition(0.0, 0.0, 0.8);
      scs.setCameraPosition(0.0, 5.0, 2.0);
      scs.addYoGraphics(SCS1GraphicConversionTools.toYoGraphicDefinitions(controller.getYoGraphicsListRegistry()));
      scs.requestCameraRigidBodyTracking(scs.getRobots().get(0).getName(), scs.getRobots().get(0).getAllJoints().get(0).getSuccessor().getName());
      scs.requestPlotter2DCoordinateTracking("measuredCenterOfMassX","measuredCenterOfMassY","worldFrame");
      scs.showOverheadPlotter2D(true);
      scs.start(false, false, false);
   }

   public static void main(String[] args)
   {
      LIPMWalkerSimulationSCS2 simulation = new LIPMWalkerSimulationSCS2();
   }
}
