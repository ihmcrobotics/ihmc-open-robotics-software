package us.ihmc.humanoidBehaviors;

import java.io.IOException;
import java.util.Arrays;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.ObjectDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.BasicPipeLineBehavior;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.BasicStateMachineBehavior;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.FireFighterStanceBehavior;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.PickUpBallBehaviorStateMachine;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.RepeatedlyWalkFootstepListBehavior;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.ResetRobotBehavior;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.TurnValveBehaviorStateMachine;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkThroughDoorBehavior;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkToGoalBehavior;
import us.ihmc.humanoidBehaviors.behaviors.debug.PartialFootholdBehavior;
import us.ihmc.humanoidBehaviors.behaviors.debug.TestGarbageGenerationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.debug.TestICPOptimizationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.debug.TestSmoothICPPlannerBehavior;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.DiagnosticBehavior;
import us.ihmc.humanoidBehaviors.behaviors.examples.ExampleComplexBehaviorStateMachine;
import us.ihmc.humanoidBehaviors.behaviors.fiducialLocation.FollowFiducialBehavior;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.LocateGoalBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.CollaborativeBehavior;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.PushAndWalkBehavior;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.WalkOverTerrainStateMachineBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorControlModeSubscriber;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorDispatcher;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorTypeSubscriber;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class IHMCHumanoidBehaviorManager
{
   public static final double BEHAVIOR_YO_VARIABLE_SERVER_DT = 0.01;

   private static double runAutomaticDiagnosticTimeToWait = Double.NaN;

   private final PacketCommunicator behaviorPacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.BEHAVIOUR_MODULE_PORT,
         new IHMCCommunicationKryoNetClassList());

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble yoTime = new YoDouble("yoTime", registry);

   private YoVariableServer yoVariableServer = null;

   public IHMCHumanoidBehaviorManager(WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModelFactory robotModelFactory,
                                      LogModelProvider modelProvider, boolean startYoVariableServer, DRCRobotSensorInformation sensorInfo)
         throws IOException
   {
      this(wholeBodyControllerParameters, robotModelFactory, modelProvider, startYoVariableServer, sensorInfo, false);
   }

   public static void setAutomaticDiagnosticTimeToWait(double timeToWait)
   {
      runAutomaticDiagnosticTimeToWait = timeToWait;
   }

   private IHMCHumanoidBehaviorManager(WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModelFactory robotModelFactory,
                                       LogModelProvider modelProvider, boolean startYoVariableServer, DRCRobotSensorInformation sensorInfo,
                                       boolean runAutomaticDiagnostic)
         throws IOException
   {
      System.out.println(PrintTools.INFO + getClass().getSimpleName() + ": Initializing");

      if (startYoVariableServer)
      {
         PeriodicThreadSchedulerFactory scheduler = new PeriodicNonRealtimeThreadSchedulerFactory();
         yoVariableServer = new YoVariableServer(getClass(), scheduler, modelProvider, LogSettings.BEHAVIOR, BEHAVIOR_YO_VARIABLE_SERVER_DT);
      }

      FullHumanoidRobotModel fullRobotModel = robotModelFactory.createFullRobotModel();
      CommunicationBridge communicationBridge = new CommunicationBridge(behaviorPacketCommunicator);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(false);
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      HumanoidRobotDataReceiver robotDataReceiver = new HumanoidRobotDataReceiver(fullRobotModel, forceSensorDataHolder);

      HumanoidReferenceFrames referenceFrames = robotDataReceiver.getReferenceFrames();
      behaviorPacketCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      BehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new BehaviorControlModeSubscriber();
      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();

      BehaviorDispatcher<HumanoidBehaviorType> dispatcher = new BehaviorDispatcher<>(yoTime, robotDataReceiver, desiredBehaviorControlSubscriber,
            desiredBehaviorSubscriber, communicationBridge, yoVariableServer, HumanoidBehaviorType.class, HumanoidBehaviorType.STOP, registry,
            yoGraphicsListRegistry);

      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      behaviorPacketCommunicator.attachListener(CapturabilityBasedStatus.class, capturabilityBasedStatusSubsrciber);

      CapturePointUpdatable capturePointUpdatable = new CapturePointUpdatable(capturabilityBasedStatusSubsrciber, yoGraphicsListRegistry, registry);
      dispatcher.addUpdatable(capturePointUpdatable);

      //      YoDouble minIcpDistanceToSupportPolygon = capturePointUpdatable.getMinIcpDistanceToSupportPolygon();
      //      YoDouble icpError = capturePointUpdatable.getIcpError();

      SideDependentList<WristForceSensorFilteredUpdatable> wristSensorUpdatables = null;
      if (sensorInfo.getWristForceSensorNames() != null && !sensorInfo.getWristForceSensorNames().containsValue(null))
      {
         wristSensorUpdatables = new SideDependentList<>();
         for (RobotSide robotSide : RobotSide.values)
         {
            WristForceSensorFilteredUpdatable wristSensorUpdatable = new WristForceSensorFilteredUpdatable(robotSide, fullRobotModel, sensorInfo,
                  forceSensorDataHolder, BEHAVIOR_YO_VARIABLE_SERVER_DT, behaviorPacketCommunicator, registry);
            wristSensorUpdatables.put(robotSide, wristSensorUpdatable);
            dispatcher.addUpdatable(wristSensorUpdatable);
         }
      }

      if (runAutomaticDiagnostic && !Double.isNaN(runAutomaticDiagnosticTimeToWait) && !Double.isInfinite(runAutomaticDiagnosticTimeToWait))
      {
         createAndRegisterAutomaticDiagnostic(dispatcher, fullRobotModel, referenceFrames, yoTime, communicationBridge, capturePointUpdatable,
                                              wholeBodyControllerParameters, runAutomaticDiagnosticTimeToWait, yoGraphicsListRegistry);
      }
      else
      {
         createAndRegisterBehaviors(dispatcher, modelProvider, fullRobotModel, robotModelFactory, wristSensorUpdatables, referenceFrames, yoTime,
                                    communicationBridge, yoGraphicsListRegistry, capturePointUpdatable, wholeBodyControllerParameters);
      }

      behaviorPacketCommunicator.attachListener(BehaviorControlModePacket.class, desiredBehaviorControlSubscriber);
      behaviorPacketCommunicator.attachListener(HumanoidBehaviorTypePacket.class, desiredBehaviorSubscriber);

      behaviorPacketCommunicator.connect();

      if (startYoVariableServer)
      {
         yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), yoGraphicsListRegistry);
         yoVariableServer.start();
      }

      dispatcher.start();
   }

   /**
    * Create the different behaviors and register them in the dispatcher.
    * When creating a new behavior, that's where you need to add it.
    * @param fullRobotModel Holds the robot data (like joint angles). The data is updated in the dispatcher and can be shared with the behaviors.
    * @param referenceFrames Give access to useful references related to the robot. They're automatically updated.
    * @param yoTime Holds the controller time. It is updated in the dispatcher and can be shared with the behaviors.
    * @param behaviorCommunicationBridge used to send packets to the controller.
    * @param yoGraphicsListRegistry Allows to register YoGraphics that will be displayed in SCS.
    * @param wholeBodyControllerParameters
    * @param wristSensors Holds the force sensor data
    */
   private void createAndRegisterBehaviors(BehaviorDispatcher<HumanoidBehaviorType> dispatcher,
         LogModelProvider logModelProvider, FullHumanoidRobotModel fullRobotModel, FullHumanoidRobotModelFactory robotModelFactory,
         SideDependentList<WristForceSensorFilteredUpdatable> wristSensors, HumanoidReferenceFrames referenceFrames, YoDouble yoTime,
         CommunicationBridge behaviorCommunicationBridge, YoGraphicsListRegistry yoGraphicsListRegistry, CapturePointUpdatable capturePointUpdatable,
         WholeBodyControllerParameters wholeBodyControllerParameters)
   {

      WalkingControllerParameters walkingControllerParameters = wholeBodyControllerParameters.getWalkingControllerParameters();
      AtlasPrimitiveActions atlasPrimitiveActions = new AtlasPrimitiveActions(behaviorCommunicationBridge, fullRobotModel, robotModelFactory, referenceFrames,
                                                                              yoTime, wholeBodyControllerParameters, registry);
      YoBoolean yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();
      YoEnum<RobotSide> yoSupportLeg = capturePointUpdatable.getYoSupportLeg();
      YoFrameConvexPolygon2d yoSupportPolygon = capturePointUpdatable.getYoSupportPolygon();

      // CREATE SERVICES
      FiducialDetectorBehaviorService fiducialDetectorBehaviorService = new FiducialDetectorBehaviorService(behaviorCommunicationBridge, yoGraphicsListRegistry);
      fiducialDetectorBehaviorService.setTargetIDToLocate(50);
      dispatcher.addBehaviorService(fiducialDetectorBehaviorService);

      ObjectDetectorBehaviorService objectDetectorBehaviorService = null;
      try
      {
         objectDetectorBehaviorService = new ObjectDetectorBehaviorService(behaviorCommunicationBridge, yoGraphicsListRegistry);
         dispatcher.addBehaviorService(objectDetectorBehaviorService);
      }
      catch (Exception e)
      {
         System.err.println("Error creating valve detection behavior service");
         e.printStackTrace();
      }

      //      dispatcher.addBehavior(HumanoidBehaviorType.PICK_UP_BALL,
//            new PickUpBallBehavior(behaviorCommunicationBridge, yoTime, yoDoubleSupport, fullRobotModel, referenceFrames, wholeBodyControllerParameters));

      dispatcher.addBehavior(HumanoidBehaviorType.FIRE_FIGHTING,
            new FireFighterStanceBehavior("fireFighting", yoTime,behaviorCommunicationBridge, fullRobotModel, referenceFrames, wholeBodyControllerParameters, atlasPrimitiveActions));



      dispatcher.addBehavior(HumanoidBehaviorType.PICK_UP_BALL,
            new PickUpBallBehaviorStateMachine(behaviorCommunicationBridge, yoTime, yoDoubleSupport, fullRobotModel, referenceFrames, wholeBodyControllerParameters, atlasPrimitiveActions));

      dispatcher.addBehavior(HumanoidBehaviorType.RESET_ROBOT,
            new ResetRobotBehavior(behaviorCommunicationBridge, yoTime));


      dispatcher.addBehavior(HumanoidBehaviorType.TURN_VALVE,
            new TurnValveBehaviorStateMachine(behaviorCommunicationBridge, yoTime, yoDoubleSupport, fullRobotModel, referenceFrames, wholeBodyControllerParameters, atlasPrimitiveActions));

      dispatcher.addBehavior(HumanoidBehaviorType.WALK_THROUGH_DOOR,
            new WalkThroughDoorBehavior(behaviorCommunicationBridge, yoTime, yoDoubleSupport, fullRobotModel, referenceFrames, wholeBodyControllerParameters, atlasPrimitiveActions));

      dispatcher.addBehavior(HumanoidBehaviorType.DEBUG_PARTIAL_FOOTHOLDS,
            new PartialFootholdBehavior(behaviorCommunicationBridge));


      dispatcher.addBehavior(HumanoidBehaviorType.TEST_ICP_OPTIMIZATION,
            new TestICPOptimizationBehavior(behaviorCommunicationBridge, referenceFrames, yoTime));

      dispatcher.addBehavior(HumanoidBehaviorType.TEST_GC_GENERATION,
                             new TestGarbageGenerationBehavior(behaviorCommunicationBridge, referenceFrames, yoTime));

      dispatcher.addBehavior(HumanoidBehaviorType.TEST_SMOOTH_ICP_PLANNER,
                             new TestSmoothICPPlannerBehavior(behaviorCommunicationBridge, yoTime, yoDoubleSupport, fullRobotModel, referenceFrames, wholeBodyControllerParameters, atlasPrimitiveActions));

      dispatcher.addBehavior(HumanoidBehaviorType.PUSH_AND_WALK,
                             new PushAndWalkBehavior(behaviorCommunicationBridge, referenceFrames, fullRobotModel, walkingControllerParameters, yoGraphicsListRegistry));

      DRCRobotSensorInformation sensorInformation = wholeBodyControllerParameters.getSensorInformation();
      dispatcher.addBehavior(HumanoidBehaviorType.COLLABORATIVE_TASK, new CollaborativeBehavior(behaviorCommunicationBridge, referenceFrames, fullRobotModel, sensorInformation, walkingControllerParameters, yoGraphicsListRegistry));

      dispatcher.addBehavior(HumanoidBehaviorType.EXAMPLE_BEHAVIOR,
            new ExampleComplexBehaviorStateMachine(behaviorCommunicationBridge, yoTime, atlasPrimitiveActions));

      dispatcher.addBehavior(HumanoidBehaviorType.LOCATE_FIDUCIAL, new LocateGoalBehavior(behaviorCommunicationBridge, fiducialDetectorBehaviorService));
      dispatcher.addBehavior(HumanoidBehaviorType.FOLLOW_FIDUCIAL_50, new FollowFiducialBehavior(behaviorCommunicationBridge, fullRobotModel, referenceFrames, fiducialDetectorBehaviorService));
      dispatcher.addBehavior(HumanoidBehaviorType.WAlK_OVER_TERRAIN, new WalkOverTerrainStateMachineBehavior(behaviorCommunicationBridge, yoTime, atlasPrimitiveActions, logModelProvider, fullRobotModel, referenceFrames,
                                                                                                             fiducialDetectorBehaviorService));

      if (objectDetectorBehaviorService != null)
      {
         dispatcher.addBehavior(HumanoidBehaviorType.LOCATE_VALVE, new LocateGoalBehavior(behaviorCommunicationBridge, objectDetectorBehaviorService));
         dispatcher.addBehavior(HumanoidBehaviorType.FOLLOW_VALVE,
                                new FollowFiducialBehavior(behaviorCommunicationBridge, fullRobotModel, referenceFrames, objectDetectorBehaviorService));
         dispatcher.addBehavior(HumanoidBehaviorType.WALK_OVER_TERRAIN_TO_VALVE,
                                new WalkOverTerrainStateMachineBehavior(behaviorCommunicationBridge, yoTime, atlasPrimitiveActions, logModelProvider, fullRobotModel,
                                                                        referenceFrames, objectDetectorBehaviorService));
      }

      dispatcher.addBehavior(HumanoidBehaviorType.TEST_PIPELINE,
            new BasicPipeLineBehavior("pipelineTest", yoTime, behaviorCommunicationBridge, fullRobotModel, referenceFrames, wholeBodyControllerParameters));

      dispatcher.addBehavior(HumanoidBehaviorType.TEST_STATEMACHINE,
            new BasicStateMachineBehavior("StateMachineTest", yoTime, behaviorCommunicationBridge, atlasPrimitiveActions));

      // 04/24/2017 GW: removed since this caused trouble with opencv: "Cannot load org/opencv/opencv_java320"
//      BlobFilteredSphereDetectionBehavior blobFilteredSphereDetectionBehavior = new BlobFilteredSphereDetectionBehavior(behaviorCommunicationBridge,
//            referenceFrames, fullRobotModel);
//      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_ORANGE_BALL);
//      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_BLUE_BALL);
//      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_RED_BALL);
//      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_YELLOW_BALL);
//      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_GREEN_BALL);
//      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.SIMULATED_BALL);
//      dispatcher.addBehavior(HumanoidBehaviorType.BALL_DETECTION, blobFilteredSphereDetectionBehavior);

      DiagnosticBehavior diagnosticBehavior = new DiagnosticBehavior(fullRobotModel, yoSupportLeg, referenceFrames, yoTime, yoDoubleSupport,
            behaviorCommunicationBridge, wholeBodyControllerParameters, yoSupportPolygon, yoGraphicsListRegistry);
      dispatcher.addBehavior(HumanoidBehaviorType.DIAGNOSTIC, diagnosticBehavior);

      WalkToGoalBehavior walkToGoalBehavior = new WalkToGoalBehavior(behaviorCommunicationBridge, referenceFrames, walkingControllerParameters, yoTime);
      dispatcher.addBehavior(HumanoidBehaviorType.WALK_TO_GOAL, walkToGoalBehavior);

      RepeatedlyWalkFootstepListBehavior repeatedlyWalkFootstepListBehavior = new RepeatedlyWalkFootstepListBehavior(behaviorCommunicationBridge, referenceFrames, registry);
      dispatcher.addBehavior(HumanoidBehaviorType.REPEATEDLY_WALK_FOOTSTEP_LIST, repeatedlyWalkFootstepListBehavior);
   }

   private void createAndRegisterAutomaticDiagnostic(BehaviorDispatcher<HumanoidBehaviorType> dispatcher, FullHumanoidRobotModel fullRobotModel,
         HumanoidReferenceFrames referenceFrames, YoDouble yoTime, CommunicationBridgeInterface outgoingCommunicationBridge,
         CapturePointUpdatable capturePointUpdatable, WholeBodyControllerParameters wholeBodyControllerParameters, double timeToWait,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoBoolean yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();
      YoEnum<RobotSide> yoSupportLeg = capturePointUpdatable.getYoSupportLeg();
      YoFrameConvexPolygon2d yoSupportPolygon = capturePointUpdatable.getYoSupportPolygon();

      DiagnosticBehavior diagnosticBehavior = new DiagnosticBehavior(fullRobotModel, yoSupportLeg, referenceFrames, yoTime, yoDoubleSupport,
            outgoingCommunicationBridge, wholeBodyControllerParameters, yoSupportPolygon, yoGraphicsListRegistry);
      diagnosticBehavior.setupForAutomaticDiagnostic(timeToWait);
      dispatcher.addBehavior(HumanoidBehaviorType.DIAGNOSTIC, diagnosticBehavior);
      dispatcher.requestBehavior(HumanoidBehaviorType.DIAGNOSTIC);
   }

   public static IHMCHumanoidBehaviorManager createBehaviorModuleForAutomaticDiagnostic(WholeBodyControllerParameters wholeBodyControllerParameters,
                                                                                        FullHumanoidRobotModelFactory robotModelFactory,
                                                                                        LogModelProvider modelProvider, boolean startYoVariableServer,
                                                                                        DRCRobotSensorInformation sensorInfo, double timeToWait)
         throws IOException
   {
      IHMCHumanoidBehaviorManager.setAutomaticDiagnosticTimeToWait(timeToWait);
      IHMCHumanoidBehaviorManager ihmcHumanoidBehaviorManager = new IHMCHumanoidBehaviorManager(wholeBodyControllerParameters, robotModelFactory, modelProvider,
                                                                                                startYoVariableServer, sensorInfo, true);
      return ihmcHumanoidBehaviorManager;
   }
}
