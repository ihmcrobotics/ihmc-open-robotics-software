package us.ihmc.darpaRoboticsChallenge;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelState;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.PlainDRCRobot;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectCommunicator;

import java.io.IOException;

public abstract class DRCDemo03
{
   private static final boolean START_NETWORK = true;
   private static final boolean SHOW_HEIGHTMAP = false;
   private final HumanoidRobotSimulation<SDFRobot> drcSimulation;
   private final DRCDemoEnvironmentWithBoxAndSteeringWheel environment;

   public DRCDemo03(DRCGuiInitialSetup guiInitialSetup, AutomaticSimulationRunner automaticSimulationRunner, double timePerRecordTick,
                    int simulationDataBufferSize, DRCRobotModel robotModel, DRCRobotInitialSetup<SDFRobot> robotInitialSetup, WalkingControllerParameters drivingControllerParameters, 
                    WalkingControllerParameters drcRobotMultiContactParameters)
   {
      DRCSCSInitialSetup scsInitialSetup;
      
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      
      environment = new DRCDemoEnvironmentWithBoxAndSteeringWheel(dynamicGraphicObjectsListRegistry);
      final PlainDRCRobot robotInterface = new PlainDRCRobot(robotModel);
      scsInitialSetup = new DRCSCSInitialSetup(environment, robotInterface.getSimulateDT());
      
      scsInitialSetup.setSimulationDataBufferSize(simulationDataBufferSize);
      scsInitialSetup.setInitializeEstimatorToActual(true);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(timePerRecordTick / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      environment.activateDisturbanceControllerOnSteeringWheel(YoFunctionGeneratorMode.SINE);

      ObjectCommunicator drcNetworkProcessorServer = new LocalObjectCommunicator();
      GlobalDataProducer dataProducer = new GlobalDataProducer(drcNetworkProcessorServer);

      ArmControllerParameters armControllerParameters = robotModel.getArmControllerParameters();
//      DRCRobotJointMap jointMap = robotInterface.getJointMap();
      HighLevelState initialBehavior = HighLevelState.DRIVING;
      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForFastWalkingInSimulation(drivingControllerParameters);

      
      ControllerFactory controllerFactory = DRCObstacleCourseSimulation.createDRCMultiControllerFactory(null, dataProducer, footstepTimingParameters, drivingControllerParameters, 
            armControllerParameters, drcRobotMultiContactParameters,initialBehavior,robotModel);

      
      Pair<HumanoidRobotSimulation<SDFRobot>, DRCController> humanoidSimulation = DRCSimulationFactory.createSimulation(controllerFactory, environment, robotInterface, robotInitialSetup, scsInitialSetup,
              guiInitialSetup, dataProducer, null, dynamicGraphicObjectsListRegistry,false,robotModel);
      drcSimulation = humanoidSimulation.first();

      SimulationConstructionSet simulationConstructionSet = drcSimulation.getSimulationConstructionSet();

      if (START_NETWORK)
      {
         LocalObjectCommunicator localObjectCommunicator = DRCObstacleCourseSimulation.createLocalObjectCommunicator(drcSimulation, robotInterface);

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
         planeAtZ0.addHeightMap(drcSimulation.getRobot().getGroundContactModel().getGroundProfile(), 1000, 1000, YoAppearance.Red());
         simulationConstructionSet.addStaticLinkGraphics(planeAtZ0);
      }

      setUpJoyStick(simulationConstructionSet);

      if (automaticSimulationRunner != null)
      {
         drcSimulation.start(automaticSimulationRunner);
      }
      else
      {
         drcSimulation.start(null);
      }
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
//    seatGraphics.addModelFile(DRCDemo03.class.getResource("models/seat.3DS"));
//    sim.addStaticLinkGraphics(seatGraphics);
// }

   

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return drcSimulation.getSimulationConstructionSet();
   }
}
