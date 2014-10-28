package us.ihmc.darpaRoboticsChallenge;

import java.io.IOException;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemoEnvironmentWithBoxAndSteeringWheel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;
import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

import com.yobotics.simulationconstructionset.GroundContactModel;
import com.yobotics.simulationconstructionset.HeightMapFromGroundContactModel;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;

public abstract class DRCDemo03
{
   private static final boolean START_NETWORK = true;
   private static final boolean SHOW_HEIGHTMAP = false;
   private final DRCSimulationFactory drcSimulation;
   private final DRCDemoEnvironmentWithBoxAndSteeringWheel environment;

   public DRCDemo03(DRCGuiInitialSetup guiInitialSetup, DRCRobotModel robotModel, DRCRobotInitialSetup<SDFRobot> robotInitialSetup)
   {
      DRCSCSInitialSetup scsInitialSetup;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      environment = new DRCDemoEnvironmentWithBoxAndSteeringWheel(yoGraphicsListRegistry);
      scsInitialSetup = new DRCSCSInitialSetup(environment, robotModel.getSimulateDT());

      scsInitialSetup.setInitializeEstimatorToActual(true);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(robotModel.getControllerDT() / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      environment.activateDisturbanceControllerOnSteeringWheel(YoFunctionGeneratorMode.SINE);

      ObjectCommunicator drcNetworkProcessorServer = new LocalObjectCommunicator();
      GlobalDataProducer dataProducer = new GlobalDataProducer(drcNetworkProcessorServer);

      HighLevelState initialBehavior = HighLevelState.DRIVING;
      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters
            .createForFastWalkingInSimulation(robotModel.getDrivingControllerParameters());

      MomentumBasedControllerFactory controllerFactory = DRCObstacleCourseSimulation.createDRCMultiControllerFactory(null, dataProducer,
            footstepTimingParameters, initialBehavior, robotModel);

      drcSimulation = new DRCSimulationFactory(robotModel, controllerFactory, environment, robotInitialSetup, scsInitialSetup, guiInitialSetup, dataProducer);

      SimulationConstructionSet simulationConstructionSet = drcSimulation.getSimulationConstructionSet();

      if (START_NETWORK)
      {
         LocalObjectCommunicator localObjectCommunicator = DRCObstacleCourseSimulation.createLocalObjectCommunicator(drcSimulation, robotModel);

         new DRCNetworkProcessor(localObjectCommunicator, drcNetworkProcessorServer, robotModel);

         try
         {
            drcNetworkProcessorServer.connect();
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
      }

      simulationConstructionSet.setCameraPosition(6.0, -2.0, 4.5);
      simulationConstructionSet.setCameraFix(-0.44, -0.17, 0.75);

      //    showSeatGraphics(simulationConstructionSet);

      if (SHOW_HEIGHTMAP)
      {
         Graphics3DObject planeAtZ0 = new Graphics3DObject();

         HeightMap heightMap = null;
         GroundContactModel groundContactModel = drcSimulation.getRobot().getGroundContactModel();

         if (groundContactModel != null)
         {
            heightMap = HeightMapFromGroundContactModel.getHeightMap(groundContactModel);
         }

         planeAtZ0.addHeightMap(heightMap, 1000, 1000, YoAppearance.Red());
         simulationConstructionSet.addStaticLinkGraphics(planeAtZ0);
      }

      setUpJoyStick(simulationConstructionSet);

      drcSimulation.start();
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

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return drcSimulation.getSimulationConstructionSet();
   }
}
