package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.robot.Robot;

public class RaymanWalkerSimulation
{
   private static final double simDT = 0.0004;
   private static final double controlDT = simDT * 100;

   public RaymanWalkerSimulation()
   {
      RaymanDefinition walkerDef = new RaymanDefinition();

      // Define ground contact parameters
      ContactPointBasedContactParameters parameters = ContactPointBasedContactParameters.defaultParameters();
      parameters.setKxy(4000.0);
      parameters.setBxy(100.0);
      parameters.setKz(150.0);
      parameters.setBz(50.0);
//      parameters.setKz(500.0);
//      parameters.setBz(250.0);

      // Instantiate a SCS object
      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.contactPointBasedPhysicsEngineFactory(parameters));

      // The gravity has to be explicitly defined for the controller core (maybe a robot on the Moon someday...?)
      double gravityMagnitude = 9.81;

      // Make sure the simulation and the controller are using the same value for the gravity.
      scs.getGravity().set(0.0, 0.0, -gravityMagnitude);

      // This time, we will make the controller run at a slower frequency than the simulation.

      // The simulation time step.
      scs.setDT(simDT);
      // Set the frequency at which data is logged.
      scs.setBufferRecordTickPeriod(10);
      scs.changeBufferSize(100000);
      scs.setRealTimeRateSimulation(true);

      // Generate a robot for the simulation
      Robot walker = scs.addRobot(walkerDef);

      // Add a terrain
      scs.addTerrainObject(new GroundDefinition());

      RaymanController controller = new RaymanController(walker.getControllerInput(), walker.getControllerOutput(), controlDT, gravityMagnitude, walkerDef);

      controller.initialize();
//      scs.addYoEntry("walk");
//      scs.addYoEntry("measuredCapturePointX");
//      scs.addYoEntry("measuredCapturePointY");
//      scs.addYoEntry("measuredCapturePointZ");


      walker.addController(controller);

      // Camera settings
      scs.setCameraFocusPosition(0.0, 0.0, 0.8);
      scs.setCameraPosition(0.0, 5.0, 2.0);

      scs.addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(controller.getYoGraphicsListRegistry()));
      scs.addYoGraphic(controller.getYoGraphicDefinition());
      scs.requestCameraRigidBodyTracking(scs.getRobots().get(0).getName(), scs.getRobots().get(0).getAllJoints().get(0).getSuccessor().getName());
      scs.requestPlotter2DCoordinateTracking("measuredCenterOfMassPointX","measuredCenterOfMassPointY","worldFrame");
      scs.showOverheadPlotter2D(true);
      scs.start(false, false, false);
   }

   public static void main(String[] args)
   {
      RaymanWalkerSimulation simulation = new RaymanWalkerSimulation();
   }
}
