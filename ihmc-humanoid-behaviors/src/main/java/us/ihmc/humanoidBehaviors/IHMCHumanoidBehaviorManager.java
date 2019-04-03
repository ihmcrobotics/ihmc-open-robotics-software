package us.ihmc.humanoidBehaviors;

import java.io.IOException;
import java.util.Arrays;

import controller_msgs.msg.dds.BehaviorControlModePacket;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.HumanoidBehaviorTypePacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
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
import us.ihmc.humanoidBehaviors.behaviors.primitives.BasicTimingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationPlannedBehavior;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.CollaborativeBehavior;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.WalkOverTerrainStateMachineBehavior;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorControlModeSubscriber;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorDispatcher;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorTypeSubscriber;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;

public class IHMCHumanoidBehaviorManager
{
   public static final double BEHAVIOR_YO_VARIABLE_SERVER_DT = 0.01;

   private static double runAutomaticDiagnosticTimeToWait = Double.NaN;

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ihmc_humanoid_behavior_node");

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble yoTime = new YoDouble("yoTime", registry);

   private YoVariableServer yoVariableServer = null;

   public IHMCHumanoidBehaviorManager(String robotName, WholeBodyControllerParameters wholeBodyControllerParameters,
                                      FullHumanoidRobotModelFactory robotModelFactory, LogModelProvider modelProvider, boolean startYoVariableServer,
                                      DRCRobotSensorInformation sensorInfo)
         throws IOException
   {
      this(robotName, wholeBodyControllerParameters, robotModelFactory, modelProvider, startYoVariableServer, sensorInfo, false);
   }

   public static void setAutomaticDiagnosticTimeToWait(double timeToWait)
   {
      runAutomaticDiagnosticTimeToWait = timeToWait;
   }

   private IHMCHumanoidBehaviorManager(String robotName, WholeBodyControllerParameters wholeBodyControllerParameters,
                                       FullHumanoidRobotModelFactory robotModelFactory, LogModelProvider modelProvider, boolean startYoVariableServer,
                                       DRCRobotSensorInformation sensorInfo, boolean runAutomaticDiagnostic)
         throws IOException
   {
      System.out.println(PrintTools.INFO + getClass().getSimpleName() + ": Initializing");

      if (startYoVariableServer)
      {
         yoVariableServer = new YoVariableServer(getClass(), modelProvider, LogSettings.BEHAVIOR, BEHAVIOR_YO_VARIABLE_SERVER_DT);
      }

      FullHumanoidRobotModel fullRobotModel = robotModelFactory.createFullRobotModel();

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(false);
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      HumanoidRobotDataReceiver robotDataReceiver = new HumanoidRobotDataReceiver(fullRobotModel, forceSensorDataHolder);

      HumanoidReferenceFrames referenceFrames = robotDataReceiver.getReferenceFrames();

      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);

      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, controllerPubGenerator,
                                           s -> robotDataReceiver.receivedPacket(s.takeNextData()));

      BehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new BehaviorControlModeSubscriber();
      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();

      BehaviorDispatcher<HumanoidBehaviorType> dispatcher = new BehaviorDispatcher<>(robotName, yoTime, robotDataReceiver, desiredBehaviorControlSubscriber,
                                                                                     desiredBehaviorSubscriber, ros2Node, yoVariableServer,
                                                                                     HumanoidBehaviorType.class, HumanoidBehaviorType.STOP, registry,
                                                                                     yoGraphicsListRegistry);

      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      ROS2Tools.createCallbackSubscription(ros2Node, CapturabilityBasedStatus.class, controllerPubGenerator,
                                           s -> capturabilityBasedStatusSubsrciber.receivedPacket(s.takeNextData()));

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
            WristForceSensorFilteredUpdatable wristSensorUpdatable = new WristForceSensorFilteredUpdatable(robotName, robotSide, fullRobotModel, sensorInfo,
                                                                                                           forceSensorDataHolder,
                                                                                                           BEHAVIOR_YO_VARIABLE_SERVER_DT, ros2Node, registry);
            wristSensorUpdatables.put(robotSide, wristSensorUpdatable);
            dispatcher.addUpdatable(wristSensorUpdatable);
         }
      }

      if (runAutomaticDiagnostic && !Double.isNaN(runAutomaticDiagnosticTimeToWait) && !Double.isInfinite(runAutomaticDiagnosticTimeToWait))
      {
         createAndRegisterAutomaticDiagnostic(robotName, dispatcher, fullRobotModel, referenceFrames, yoTime, ros2Node, capturePointUpdatable,
                                              wholeBodyControllerParameters, runAutomaticDiagnosticTimeToWait, yoGraphicsListRegistry);
      }
      else
      {
         createAndRegisterBehaviors(robotName, dispatcher, modelProvider, fullRobotModel, robotModelFactory, wristSensorUpdatables, referenceFrames, yoTime,
                                    ros2Node, yoGraphicsListRegistry, capturePointUpdatable, wholeBodyControllerParameters);
      }

      MessageTopicNameGenerator behaviorSubGenerator = getSubscriberTopicNameGenerator(robotName);
      dispatcher.finalizeStateMachine();
      ROS2Tools.createCallbackSubscription(ros2Node, BehaviorControlModePacket.class, behaviorSubGenerator,
                                           s -> desiredBehaviorControlSubscriber.receivedPacket(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, HumanoidBehaviorTypePacket.class, behaviorSubGenerator,
                                           s -> desiredBehaviorSubscriber.receivedPacket(s.takeNextData()));

      if (startYoVariableServer)
      {
         yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), yoGraphicsListRegistry);
         yoVariableServer.start();
      }

      dispatcher.start();
   }

   /**
    * Create the different behaviors and register them in the dispatcher. When creating a new
    * behavior, that's where you need to add it.
    * 
    * @param robotName TODO
    * @param fullRobotModel Holds the robot data (like joint angles). The data is updated in the
    *           dispatcher and can be shared with the behaviors.
    * @param wristSensors Holds the force sensor data
    * @param referenceFrames Give access to useful references related to the robot. They're
    *           automatically updated.
    * @param yoTime Holds the controller time. It is updated in the dispatcher and can be shared
    *           with the behaviors.
    * @param ros2Node used to send packets to the controller.
    * @param yoGraphicsListRegistry Allows to register YoGraphics that will be displayed in SCS.
    * @param wholeBodyControllerParameters
    */
   private void createAndRegisterBehaviors(String robotName, BehaviorDispatcher<HumanoidBehaviorType> dispatcher, LogModelProvider logModelProvider,
                                           FullHumanoidRobotModel fullRobotModel, FullHumanoidRobotModelFactory robotModelFactory,
                                           SideDependentList<WristForceSensorFilteredUpdatable> wristSensors, HumanoidReferenceFrames referenceFrames,
                                           YoDouble yoTime, Ros2Node ros2Node, YoGraphicsListRegistry yoGraphicsListRegistry,
                                           CapturePointUpdatable capturePointUpdatable, WholeBodyControllerParameters wholeBodyControllerParameters)
   {

      WalkingControllerParameters walkingControllerParameters = wholeBodyControllerParameters.getWalkingControllerParameters();
      AtlasPrimitiveActions atlasPrimitiveActions = new AtlasPrimitiveActions(robotName, ros2Node, fullRobotModel, robotModelFactory, referenceFrames, yoTime,
                                                                              wholeBodyControllerParameters, registry);
      YoBoolean yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();
      YoEnum<RobotSide> yoSupportLeg = capturePointUpdatable.getYoSupportLeg();
      YoFrameConvexPolygon2D yoSupportPolygon = capturePointUpdatable.getYoSupportPolygon();

      // CREATE SERVICES
      FiducialDetectorBehaviorService fiducialDetectorBehaviorService = new FiducialDetectorBehaviorService(robotName, ros2Node, yoGraphicsListRegistry);
      fiducialDetectorBehaviorService.setTargetIDToLocate(50);
      dispatcher.addBehaviorService(fiducialDetectorBehaviorService);

      ObjectDetectorBehaviorService objectDetectorBehaviorService = null;
      try
      {
         objectDetectorBehaviorService = new ObjectDetectorBehaviorService(robotName, ros2Node, yoGraphicsListRegistry);
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
                             new FireFighterStanceBehavior(robotName, "fireFighting", yoTime, ros2Node, fullRobotModel, referenceFrames,
                                                           wholeBodyControllerParameters, atlasPrimitiveActions));

      dispatcher.addBehavior(HumanoidBehaviorType.PICK_UP_BALL,
                             new PickUpBallBehaviorStateMachine(robotName, ros2Node, yoTime, yoDoubleSupport, fullRobotModel, referenceFrames,
                                                                wholeBodyControllerParameters, atlasPrimitiveActions));

      dispatcher.addBehavior(HumanoidBehaviorType.RESET_ROBOT, new ResetRobotBehavior(robotName, ros2Node, yoTime));
      
      dispatcher.addBehavior(HumanoidBehaviorType.BASIC_TIMER_BEHAVIOR, new BasicTimingBehavior(robotName, ros2Node));

      dispatcher.addBehavior(HumanoidBehaviorType.TURN_VALVE,
                             new TurnValveBehaviorStateMachine(robotName, ros2Node, yoTime, yoDoubleSupport, fullRobotModel, referenceFrames,
                                                               wholeBodyControllerParameters, atlasPrimitiveActions));

      dispatcher.addBehavior(HumanoidBehaviorType.WALK_THROUGH_DOOR,
                             new WalkThroughDoorBehavior(robotName, ros2Node, yoTime, yoDoubleSupport, fullRobotModel, referenceFrames,
                                                         wholeBodyControllerParameters, atlasPrimitiveActions));

      dispatcher.addBehavior(HumanoidBehaviorType.DEBUG_PARTIAL_FOOTHOLDS, new PartialFootholdBehavior(robotName, ros2Node));

      dispatcher.addBehavior(HumanoidBehaviorType.TEST_ICP_OPTIMIZATION, new TestICPOptimizationBehavior(robotName, ros2Node, referenceFrames, yoTime));

      dispatcher.addBehavior(HumanoidBehaviorType.TEST_GC_GENERATION, new TestGarbageGenerationBehavior(robotName, ros2Node, referenceFrames, yoTime));

      dispatcher.addBehavior(HumanoidBehaviorType.TEST_SMOOTH_ICP_PLANNER,
                             new TestSmoothICPPlannerBehavior(robotName, ros2Node, yoTime, yoDoubleSupport, fullRobotModel, referenceFrames,
                                                              wholeBodyControllerParameters, atlasPrimitiveActions));

      DRCRobotSensorInformation sensorInformation = wholeBodyControllerParameters.getSensorInformation();
      dispatcher.addBehavior(HumanoidBehaviorType.COLLABORATIVE_TASK,
                             new CollaborativeBehavior(robotName, ros2Node, referenceFrames, fullRobotModel, sensorInformation, walkingControllerParameters,
                                                       yoGraphicsListRegistry));

      dispatcher.addBehavior(HumanoidBehaviorType.EXAMPLE_BEHAVIOR, new ExampleComplexBehaviorStateMachine(robotName, ros2Node, yoTime, atlasPrimitiveActions));

      dispatcher.addBehavior(HumanoidBehaviorType.LOCATE_FIDUCIAL, new LocateGoalBehavior(robotName, ros2Node, fiducialDetectorBehaviorService));
      dispatcher.addBehavior(HumanoidBehaviorType.FOLLOW_FIDUCIAL_50,
                             new FollowFiducialBehavior(robotName, ros2Node, fullRobotModel, referenceFrames, fiducialDetectorBehaviorService));
      dispatcher.addBehavior(HumanoidBehaviorType.WALK_OVER_TERRAIN,
                             new WalkOverTerrainStateMachineBehavior(robotName, ros2Node, yoTime, wholeBodyControllerParameters, referenceFrames));

      if (objectDetectorBehaviorService != null)
      {
         dispatcher.addBehavior(HumanoidBehaviorType.LOCATE_VALVE, new LocateGoalBehavior(robotName, ros2Node, objectDetectorBehaviorService));
         dispatcher.addBehavior(HumanoidBehaviorType.FOLLOW_VALVE,
                                new FollowFiducialBehavior(robotName, ros2Node, fullRobotModel, referenceFrames, objectDetectorBehaviorService));
      }

      dispatcher.addBehavior(HumanoidBehaviorType.TEST_PIPELINE, new BasicPipeLineBehavior(robotName, "pipelineTest", yoTime, ros2Node, fullRobotModel,
                                                                                           referenceFrames, wholeBodyControllerParameters));

      dispatcher.addBehavior(HumanoidBehaviorType.TEST_STATEMACHINE,
                             new BasicStateMachineBehavior(robotName, "StateMachineTest", yoTime, ros2Node, atlasPrimitiveActions));

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

      DiagnosticBehavior diagnosticBehavior = new DiagnosticBehavior(robotName, fullRobotModel, yoSupportLeg, referenceFrames, yoTime, yoDoubleSupport,
                                                                     ros2Node, wholeBodyControllerParameters, yoSupportPolygon, yoGraphicsListRegistry);
      diagnosticBehavior.setCanArmsReachFarBehind(robotModelFactory.getRobotDescription().getName().contains("valkyrie"));
      dispatcher.addBehavior(HumanoidBehaviorType.DIAGNOSTIC, diagnosticBehavior);

      WalkToGoalBehavior walkToGoalBehavior = new WalkToGoalBehavior(robotName, ros2Node, referenceFrames, walkingControllerParameters, yoTime);
      dispatcher.addBehavior(HumanoidBehaviorType.WALK_TO_GOAL, walkToGoalBehavior);
      
      WalkToLocationPlannedBehavior walkToLocationBehavior = new WalkToLocationPlannedBehavior(robotName, ros2Node, fullRobotModel, referenceFrames, walkingControllerParameters, yoTime);
      dispatcher.addBehavior(HumanoidBehaviorType.WALK_TO_LOCATION, walkToLocationBehavior);

      RepeatedlyWalkFootstepListBehavior repeatedlyWalkFootstepListBehavior = new RepeatedlyWalkFootstepListBehavior(robotName, ros2Node, referenceFrames,
                                                                                                                     registry);
      dispatcher.addBehavior(HumanoidBehaviorType.REPEATEDLY_WALK_FOOTSTEP_LIST, repeatedlyWalkFootstepListBehavior);
   }

   private void createAndRegisterAutomaticDiagnostic(String robotName, BehaviorDispatcher<HumanoidBehaviorType> dispatcher,
                                                     FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, YoDouble yoTime,
                                                     Ros2Node ros2Node, CapturePointUpdatable capturePointUpdatable,
                                                     WholeBodyControllerParameters wholeBodyControllerParameters, double timeToWait,
                                                     YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoBoolean yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();
      YoEnum<RobotSide> yoSupportLeg = capturePointUpdatable.getYoSupportLeg();
      YoFrameConvexPolygon2D yoSupportPolygon = capturePointUpdatable.getYoSupportPolygon();

      DiagnosticBehavior diagnosticBehavior = new DiagnosticBehavior(robotName, fullRobotModel, yoSupportLeg, referenceFrames, yoTime, yoDoubleSupport,
                                                                     ros2Node, wholeBodyControllerParameters, yoSupportPolygon, yoGraphicsListRegistry);
      diagnosticBehavior.setupForAutomaticDiagnostic(timeToWait);
      dispatcher.addBehavior(HumanoidBehaviorType.DIAGNOSTIC, diagnosticBehavior);
      dispatcher.requestBehavior(HumanoidBehaviorType.DIAGNOSTIC);
   }

   public static IHMCHumanoidBehaviorManager createBehaviorModuleForAutomaticDiagnostic(String robotName,
                                                                                        WholeBodyControllerParameters wholeBodyControllerParameters,
                                                                                        FullHumanoidRobotModelFactory robotModelFactory,
                                                                                        LogModelProvider modelProvider, boolean startYoVariableServer,
                                                                                        DRCRobotSensorInformation sensorInfo, double timeToWait)
         throws IOException
   {
      IHMCHumanoidBehaviorManager.setAutomaticDiagnosticTimeToWait(timeToWait);
      IHMCHumanoidBehaviorManager ihmcHumanoidBehaviorManager = new IHMCHumanoidBehaviorManager(robotName, wholeBodyControllerParameters, robotModelFactory,
                                                                                                modelProvider, startYoVariableServer, sensorInfo, true);
      return ihmcHumanoidBehaviorManager;
   }

   public static String getBehaviorRosTopicPrefix(String robotName, ROS2TopicQualifier qualifier)
   {
      return ROS2Tools.IHMC_ROS_TOPIC_PREFIX + "/" + robotName.toLowerCase() + ROS2Tools.BEHAVIOR_MODULE + qualifier.toString();
   }

   public static String getBehaviorOutputRosTopicPrefix(String robotName)
   {
      return getBehaviorRosTopicPrefix(robotName, ROS2TopicQualifier.OUTPUT);
   }

   public static String getBehaviorInputRosTopicPrefix(String robotName)
   {
      return getBehaviorRosTopicPrefix(robotName, ROS2TopicQualifier.INPUT);
   }

   public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.BEHAVIOR_MODULE, ROS2TopicQualifier.OUTPUT);
   }

   public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.BEHAVIOR_MODULE, ROS2TopicQualifier.INPUT);
   }
}
