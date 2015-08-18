package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemoEnvironmentWithBoxAndSteeringWheel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.HeightMapFromGroundContactModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public abstract class DRCDemo03
{
   private static final boolean SHOW_HEIGHTMAP = false;
   private final DRCDemoEnvironmentWithBoxAndSteeringWheel environment;

   public DRCDemo03(DRCGuiInitialSetup guiInitialSetup, DRCRobotModel robotModel, DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup)
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      environment = new DRCDemoEnvironmentWithBoxAndSteeringWheel(yoGraphicsListRegistry);
      environment.activateDisturbanceControllerOnSteeringWheel(YoFunctionGeneratorMode.SINE);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, environment);
      simulationStarter.setRunMultiThreaded(true);
      
      simulationStarter.setGuiInitialSetup(guiInitialSetup);
      simulationStarter.setRobotInitialSetup(robotInitialSetup);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.setSCSCameraPosition(6.0, -2.0, 4.5);
      simulationStarter.setSCSCameraFix(-0.44, -0.17, 0.75);

      SimulationConstructionSet simulationConstructionSet = simulationStarter.getSimulationConstructionSet();

      if (SHOW_HEIGHTMAP)
      {
         Graphics3DObject planeAtZ0 = new Graphics3DObject();

         HeightMap heightMap = null;
         GroundContactModel groundContactModel = simulationStarter.getSDFRobot().getGroundContactModel();

         if (groundContactModel != null)
         {
            heightMap = HeightMapFromGroundContactModel.getHeightMap(groundContactModel);
         }

         planeAtZ0.addHeightMap(heightMap, 1000, 1000, YoAppearance.Red());
         simulationConstructionSet.addStaticLinkGraphics(planeAtZ0);
      }
      setUpJoyStick(simulationConstructionSet);

      boolean automaticallyStartSimulation = false;
      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
      networkProcessorParameters.enableUiModule(automaticallyStartSimulation);
      
      simulationStarter.startSimulation(networkProcessorParameters, automaticallyStartSimulation);
   }

   private void setUpJoyStick(SimulationConstructionSet simulationConstructionSet)
   {
      try
      {
         new DRCRobotSteeringWheelJoystickController(simulationConstructionSet);
      }
      catch (Exception e)
      {
         System.out.println("Could not connect to joystick");
      }
   }

   // private void showSeatGraphics(SimulationConstructionSet sim)
   // {
   //    Graphics3DObject seatGraphics = new Graphics3DObject();
   //    seatGraphics.scale(0.25);
   //    seatGraphics.translate(-1.25, 0, 3.25);
   //    seatGraphics.rotate(Math.toRadians(90), Graphics3DObject.Z);
   //    seatGraphics.addModelFile(DRCDemo03.class.getClassLoader().getResource("models/seat.obj"));
   //    sim.addStaticLinkGraphics(seatGraphics);
   // }
}
