package us.ihmc.humanoidBehaviors;

import java.io.IOException;
import java.util.Arrays;

import controller_msgs.msg.dds.BehaviorControlModePacket;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.HumanoidBehaviorTypePacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.*;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.DiagnosticBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorControlModeSubscriber;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorDispatcher;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorTypeSubscriber;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class IHMCHumanoidBehaviorManager implements CloseableAndDisposable
{
   public static final double BEHAVIOR_YO_VARIABLE_SERVER_DT = 0.01;

   private static double runAutomaticDiagnosticTimeToWait = Double.NaN;

   private final ROS2Node ros2Node;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble yoTime = new YoDouble("yoTime", registry);

   private YoVariableServer yoVariableServer = null;
   private FootstepPlannerParametersBasics footstepPlannerParameters;

   private final BehaviorDispatcher<HumanoidBehaviorType> dispatcher;

   public IHMCHumanoidBehaviorManager(String robotName, FootstepPlannerParametersBasics footstepPlannerParameters,
                                      WholeBodyControllerParameters<?> wholeBodyControllerParameters, FullHumanoidRobotModelFactory robotModelFactory,
                                      LogModelProvider modelProvider, boolean startYoVariableServer, HumanoidRobotSensorInformation sensorInfo)
         throws IOException
   {
      this(robotName,
           footstepPlannerParameters,
           wholeBodyControllerParameters,
           robotModelFactory,
           modelProvider,
           startYoVariableServer,
           sensorInfo,
           false,
           PubSubImplementation.FAST_RTPS);
   }

   public IHMCHumanoidBehaviorManager(String robotName,
                                      FootstepPlannerParametersBasics footstepPlannerParameters,
                                      WholeBodyControllerParameters<?> wholeBodyControllerParameters,
                                      FullHumanoidRobotModelFactory robotModelFactory,
                                      LogModelProvider modelProvider,
                                      boolean startYoVariableServer,
                                      HumanoidRobotSensorInformation sensorInfo,
                                      PubSubImplementation pubSubImplementation) throws IOException
   {

      this(robotName,
           footstepPlannerParameters,
           wholeBodyControllerParameters,
           robotModelFactory,
           modelProvider,
           startYoVariableServer,
           sensorInfo,
           false,
           pubSubImplementation);
   }

   public static void setAutomaticDiagnosticTimeToWait(double timeToWait)
   {
      runAutomaticDiagnosticTimeToWait = timeToWait;
   }

   private IHMCHumanoidBehaviorManager(String robotName,
                                       FootstepPlannerParametersBasics footstepPlannerParameters,
                                       WholeBodyControllerParameters<?> wholeBodyControllerParameters,
                                       FullHumanoidRobotModelFactory robotModelFactory,
                                       LogModelProvider modelProvider,
                                       boolean startYoVariableServer,
                                       HumanoidRobotSensorInformation sensorInfo,
                                       boolean runAutomaticDiagnostic,
                                       PubSubImplementation pubSubImplementation)
         throws IOException
   {
      LogTools.info("Initializing");
      this.footstepPlannerParameters = footstepPlannerParameters;
      if (startYoVariableServer)
      {
         yoVariableServer = new YoVariableServer(getClass(), modelProvider, LogSettings.BEHAVIOR, BEHAVIOR_YO_VARIABLE_SERVER_DT);
      }

      ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, "ihmc_humanoid_behavior_node");

      FullHumanoidRobotModel fullRobotModel = robotModelFactory.createFullRobotModel();

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(false);
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      HumanoidRobotDataReceiver robotDataReceiver = new HumanoidRobotDataReceiver(fullRobotModel, forceSensorDataHolder);

      HumanoidReferenceFrames referenceFrames = robotDataReceiver.getReferenceFrames();

      ROS2Topic controllerOutputTopic = ROS2Tools.getControllerOutputTopic(robotName);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    RobotConfigurationData.class,
                                                    controllerOutputTopic,
                                           s -> robotDataReceiver.receivedPacket(s.takeNextData()));

      BehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new BehaviorControlModeSubscriber();
      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();

      dispatcher = new BehaviorDispatcher<>(robotName,
                                            yoTime,
                                            robotDataReceiver,
                                            desiredBehaviorControlSubscriber,
                                            desiredBehaviorSubscriber,
                                            ros2Node,
                                            yoVariableServer,
                                            HumanoidBehaviorType.class,
                                            HumanoidBehaviorType.STOP,
                                            registry,
                                            yoGraphicsListRegistry);

      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    CapturabilityBasedStatus.class,
                                                    controllerOutputTopic,
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
            WristForceSensorFilteredUpdatable wristSensorUpdatable = new WristForceSensorFilteredUpdatable(robotName,
                                                                                                           robotSide,
                                                                                                           fullRobotModel,
                                                                                                           sensorInfo,
                                                                                                           forceSensorDataHolder,
                                                                                                           BEHAVIOR_YO_VARIABLE_SERVER_DT,
                                                                                                           ros2Node,
                                                                                                           registry);
            wristSensorUpdatables.put(robotSide, wristSensorUpdatable);
            dispatcher.addUpdatable(wristSensorUpdatable);
         }
      }

      if (runAutomaticDiagnostic && !Double.isNaN(runAutomaticDiagnosticTimeToWait) && !Double.isInfinite(runAutomaticDiagnosticTimeToWait))
      {
         createAndRegisterAutomaticDiagnostic(robotName,
                                              dispatcher,
                                              fullRobotModel,
                                              referenceFrames,
                                              yoTime,
                                              ros2Node,
                                              capturePointUpdatable,
                                              wholeBodyControllerParameters,
                                              footstepPlannerParameters,
                                              runAutomaticDiagnosticTimeToWait,
                                              yoGraphicsListRegistry);
      }
      else
      {
         createAndRegisterBehaviors(robotName,
                                    dispatcher,
                                    modelProvider,
                                    fullRobotModel,
                                    robotModelFactory,
                                    wristSensorUpdatables,
                                    referenceFrames,
                                    yoTime,
                                    ros2Node,
                                    yoGraphicsListRegistry,
                                    capturePointUpdatable,
                                    wholeBodyControllerParameters,
                                    footstepPlannerParameters);
      }

      ROS2Topic behaviorInputTopic = getInputTopic(robotName);
      dispatcher.finalizeStateMachine();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    BehaviorControlModePacket.class,
                                                    behaviorInputTopic,
                                           s -> desiredBehaviorControlSubscriber.receivedPacket(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    HumanoidBehaviorTypePacket.class,
                                                    behaviorInputTopic,
                                           s -> desiredBehaviorSubscriber.receivedPacket(s.takeNextData()));

      if (startYoVariableServer)
      {
         yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), yoGraphicsListRegistry);
         yoVariableServer.start();
      }

      dispatcher.start();
   }

   /**
    * Create the different behaviors and register them in the dispatcher. When creating a new behavior,
    * that's where you need to add it.
    * 
    * @param robotName                     TODO
    * @param fullRobotModel                Holds the robot data (like joint angles). The data is
    *                                      updated in the dispatcher and can be shared with the
    *                                      behaviors.
    * @param wristSensors                  Holds the force sensor data
    * @param referenceFrames               Give access to useful references related to the robot.
    *                                      They're automatically updated.
    * @param yoTime                        Holds the controller time. It is updated in the dispatcher
    *                                      and can be shared with the behaviors.
    * @param ros2Node                      used to send packets to the controller.
    * @param yoGraphicsListRegistry        Allows to register YoGraphics that will be displayed in SCS.
    * @param wholeBodyControllerParameters
    */
   private void createAndRegisterBehaviors(String robotName, BehaviorDispatcher<HumanoidBehaviorType> dispatcher, LogModelProvider logModelProvider,
                                           FullHumanoidRobotModel fullRobotModel, FullHumanoidRobotModelFactory robotModelFactory,
                                           SideDependentList<WristForceSensorFilteredUpdatable> wristSensors, HumanoidReferenceFrames referenceFrames,
                                           YoDouble yoTime, ROS2Node ros2Node, YoGraphicsListRegistry yoGraphicsListRegistry,
                                           CapturePointUpdatable capturePointUpdatable, WholeBodyControllerParameters<?> wholeBodyControllerParameters,
                                           FootstepPlannerParametersBasics footstepPlannerParameters)
   {

      WalkingControllerParameters walkingControllerParameters = wholeBodyControllerParameters.getWalkingControllerParameters();
      AtlasPrimitiveActions atlasPrimitiveActions = new AtlasPrimitiveActions(robotName,
                                                                              ros2Node,
                                                                              footstepPlannerParameters,
                                                                              fullRobotModel,
                                                                              robotModelFactory,
                                                                              referenceFrames,
                                                                              yoTime,
                                                                              wholeBodyControllerParameters,
                                                                              registry);
      YoBoolean yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();
      YoEnum<RobotSide> yoSupportLeg = capturePointUpdatable.getYoSupportLeg();
      YoFrameConvexPolygon2D yoSupportPolygon = capturePointUpdatable.getYoSupportPolygon();

      // CREATE SERVICES
//      FiducialDetectorBehaviorService fiducialDetectorBehaviorService = new FiducialDetectorBehaviorService(robotName,
//                                                                                                            FiducialDetectorBehaviorService.class.getSimpleName(),
//                                                                                                            ros2Node,
//                                                                                                            yoGraphicsListRegistry);
//      fiducialDetectorBehaviorService.setTargetIDToLocate(50);
//      fiducialDetectorBehaviorService.setExpectedFiducialSize(0.2032);
//      dispatcher.addBehaviorService(fiducialDetectorBehaviorService);

      //      ObjectDetectorBehaviorService objectDetectorBehaviorService = null;
      //      try
      //      {
      //         objectDetectorBehaviorService = new ObjectDetectorBehaviorService(robotName, ros2Node, yoGraphicsListRegistry);
      //         dispatcher.addBehaviorService(objectDetectorBehaviorService);
      //      }
      //      catch (Exception e)
      //      {
      //         System.err.println("Error creating valve detection behavior service");
      //         e.printStackTrace();
      //      }

      //      dispatcher.addBehavior(HumanoidBehaviorType.PICK_UP_BALL,
      //            new PickUpBallBehavior(behaviorCommunicationBridge, yoTime, yoDoubleSupport, fullRobotModel, referenceFrames, wholeBodyControllerParameters));

      dispatcher.addBehavior(HumanoidBehaviorType.FIRE_FIGHTING,
                             new FireFighterStanceBehavior(robotName,
                                                           "fireFighting",
                                                           yoTime,
                                                           ros2Node,
                                                           fullRobotModel,
                                                           referenceFrames,
                                                           wholeBodyControllerParameters,
                                                           atlasPrimitiveActions));
//
//      dispatcher.addBehavior(HumanoidBehaviorType.PICK_UP_BALL,
//                             new PickUpBallBehaviorStateMachine(robotName,
//                                                                ros2Node,
//                                                                yoTime,
//                                                                yoDoubleSupport,
//                                                                fullRobotModel,
//                                                                referenceFrames,
//                                                                wholeBodyControllerParameters,
//                                                                atlasPrimitiveActions));
//
      dispatcher.addBehavior(HumanoidBehaviorType.RESET_ROBOT, new ResetRobotBehavior(robotName, ros2Node, yoTime));
//
//      dispatcher.addBehavior(HumanoidBehaviorType.BASIC_TIMER_BEHAVIOR, new TimingBehaviorHelper(robotName, ros2Node));
//
//      dispatcher.addBehavior(HumanoidBehaviorType.ROUGH_TERRAIN_OPERATOR_TIMING_BEHAVIOR, new RoughTerrainTimingBehavior(robotName, yoTime, ros2Node));
//
//      dispatcher.addBehavior(HumanoidBehaviorType.TURN_VALVE,
//                             new TurnValveBehaviorStateMachine(robotName,
//                                                               ros2Node,
//                                                               yoTime,
//                                                               yoDoubleSupport,
//                                                               fullRobotModel,
//                                                               referenceFrames,
//                                                               wholeBodyControllerParameters,
//                                                               atlasPrimitiveActions));

      dispatcher.addBehavior(HumanoidBehaviorType.WALK_THROUGH_DOOR,
                             new WalkThroughDoorBehavior(robotName,
                                                         "Human",
                                                         ros2Node,
                                                         yoTime,
                                                         yoDoubleSupport,
                                                         fullRobotModel,
                                                         referenceFrames,
                                                         wholeBodyControllerParameters,
                                                         atlasPrimitiveActions,
                                                         yoGraphicsListRegistry));
      
//      dispatcher.addBehavior(HumanoidBehaviorType.WALK_THROUGH_DOOR_OPERATOR_TIMING_BEHAVIOR, new DoorTimingBehavior(robotName, yoTime, ros2Node, true));
//      dispatcher.addBehavior(HumanoidBehaviorType.WALK_THROUGH_DOOR_AUTOMATED_TIMING_BEHAVIOR,
//                             new DoorTimingBehaviorAutomated(robotName,
//                                                             ros2Node,
//                                                             yoTime,
//                                                             yoDoubleSupport,
//                                                             fullRobotModel,
//                                                             referenceFrames,
//                                                             wholeBodyControllerParameters,
//                                                             atlasPrimitiveActions,
//                                                             yoGraphicsListRegistry));
//
//      dispatcher.addBehavior(HumanoidBehaviorType.WALK_TO_LOCATION_TIMING_BEHAVIOR, new WalkTimingBehavior(robotName, yoTime, ros2Node, true));
//
//      dispatcher.addBehavior(HumanoidBehaviorType.DEBUG_PARTIAL_FOOTHOLDS, new PartialFootholdBehavior(robotName, ros2Node));
//
//      dispatcher.addBehavior(HumanoidBehaviorType.TEST_ICP_OPTIMIZATION, new TestICPOptimizationBehavior(robotName, ros2Node, referenceFrames, yoTime));
//
//      dispatcher.addBehavior(HumanoidBehaviorType.TEST_GC_GENERATION, new TestGarbageGenerationBehavior(robotName, ros2Node, referenceFrames, yoTime));
//
//      dispatcher.addBehavior(HumanoidBehaviorType.TEST_SMOOTH_ICP_PLANNER,
//                             new TestSmoothICPPlannerBehavior(robotName,
//                                                              ros2Node,
//                                                              yoTime,
//                                                              yoDoubleSupport,
//                                                              fullRobotModel,
//                                                              referenceFrames,
//                                                              wholeBodyControllerParameters,
//                                                              atlasPrimitiveActions));
//
//      HumanoidRobotSensorInformation sensorInformation = wholeBodyControllerParameters.getSensorInformation();
//      dispatcher.addBehavior(HumanoidBehaviorType.COLLABORATIVE_TASK,
//                             new CollaborativeBehavior(robotName,
//                                                       ros2Node,
//                                                       referenceFrames,
//                                                       fullRobotModel,
//                                                       sensorInformation,
//                                                       walkingControllerParameters,
//                                                       yoGraphicsListRegistry));
//
//      dispatcher.addBehavior(HumanoidBehaviorType.EXAMPLE_BEHAVIOR, new ExampleComplexBehaviorStateMachine(robotName, ros2Node, yoTime, atlasPrimitiveActions));
//
      dispatcher.addBehavior(HumanoidBehaviorType.TEST_OPENDOORDETECTOR,
                             new TestDoorOpenBehaviorService(robotName, "doorOpen", ros2Node, yoGraphicsListRegistry));
//
//      dispatcher.addBehavior(HumanoidBehaviorType.LOCATE_FIDUCIAL, new LocateGoalBehavior(robotName, ros2Node, fiducialDetectorBehaviorService));
//      dispatcher.addBehavior(HumanoidBehaviorType.FOLLOW_FIDUCIAL_50,
//                             new FollowFiducialBehavior(robotName,
//                                                        ros2Node,
//                                                        yoTime,
//                                                        wholeBodyControllerParameters,
//                                                        referenceFrames,
//                                                        fiducialDetectorBehaviorService));
//      dispatcher.addBehavior(HumanoidBehaviorType.FOLLOW_FIDUCIAL_50_AND_TURN,
//                             new WalkToFiducialAndTurnBehavior(robotName,
//                                                               ros2Node,
//                                                               yoTime,
//                                                               wholeBodyControllerParameters,
//                                                               referenceFrames,
//                                                               footstepPlannerParameters,
//                                                               fiducialDetectorBehaviorService,
//                                                               fullRobotModel));
//      dispatcher.addBehavior(HumanoidBehaviorType.WALK_OVER_TERRAIN,
//                             new WalkOverTerrainStateMachineBehavior(robotName, ros2Node, yoTime, wholeBodyControllerParameters, referenceFrames));
//
//      //      if (objectDetectorBehaviorService != null)
//      //      {
//      //         dispatcher.addBehavior(HumanoidBehaviorType.LOCATE_VALVE, new LocateGoalBehavior(robotName, ros2Node, objectDetectorBehaviorService));
//      //         dispatcher.addBehavior(HumanoidBehaviorType.FOLLOW_VALVE, new FollowFiducialBehavior(robotName, ros2Node, yoTime, wholeBodyControllerParameters,
//      //                                                                                              referenceFrames, fiducialDetectorBehaviorService));
//      //      }
//
//      dispatcher.addBehavior(HumanoidBehaviorType.TEST_PIPELINE,
//                             new BasicPipeLineBehavior(robotName,
//                                                       "pipelineTest",
//                                                       yoTime,
//                                                       ros2Node,
//                                                       fullRobotModel,
//                                                       referenceFrames,
//                                                       wholeBodyControllerParameters));
//
//      dispatcher.addBehavior(HumanoidBehaviorType.TEST_STATEMACHINE,
//                             new BasicStateMachineBehavior(robotName, "StateMachineTest", yoTime, ros2Node, atlasPrimitiveActions));
//
//      // 04/24/2017 GW: removed since this caused trouble with opencv: "Cannot load org/opencv/opencv_java320"
//      //      BlobFilteredSphereDetectionBehavior blobFilteredSphereDetectionBehavior = new BlobFilteredSphereDetectionBehavior(behaviorCommunicationBridge,
//      //            referenceFrames, fullRobotModel);
//      //      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_ORANGE_BALL);
//      //      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_BLUE_BALL);
//      //      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_RED_BALL);
//      //      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_YELLOW_BALL);
//      //      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_GREEN_BALL);
//      //      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.SIMULATED_BALL);
//      //      dispatcher.addBehavior(HumanoidBehaviorType.BALL_DETECTION, blobFilteredSphereDetectionBehavior);
//
//      DiagnosticBehavior diagnosticBehavior = new DiagnosticBehavior(robotName,
//                                                                     fullRobotModel,
//                                                                     yoSupportLeg,
//                                                                     referenceFrames,
//                                                                     yoTime,
//                                                                     yoDoubleSupport,
//                                                                     ros2Node,
//                                                                     wholeBodyControllerParameters,
//                                                                     footstepPlannerParameters,
//                                                                     yoSupportPolygon,
//                                                                     yoGraphicsListRegistry);
//      diagnosticBehavior.setCanArmsReachFarBehind(robotModelFactory.getRobotDescription().getName().contains("valkyrie"));
//      dispatcher.addBehavior(HumanoidBehaviorType.DIAGNOSTIC, diagnosticBehavior);
//
//      WalkToLocationPlannedBehavior walkToLocationBehavior = new WalkToLocationPlannedBehavior(robotName,
//                                                                                               ros2Node,
//                                                                                               fullRobotModel,
//                                                                                               referenceFrames,
//                                                                                               walkingControllerParameters,
//                                                                                               footstepPlannerParameters,
//                                                                                               yoTime);
//      dispatcher.addBehavior(HumanoidBehaviorType.WALK_TO_LOCATION, walkToLocationBehavior);
//
//      RepeatedlyWalkFootstepListBehavior repeatedlyWalkFootstepListBehavior = new RepeatedlyWalkFootstepListBehavior(robotName,
//                                                                                                                     ros2Node,
//                                                                                                                     referenceFrames,
//                                                                                                                     registry);
//      dispatcher.addBehavior(HumanoidBehaviorType.REPEATEDLY_WALK_FOOTSTEP_LIST, repeatedlyWalkFootstepListBehavior);

      LogTools.info("Finished registering behaviors.");
   }

   private void createAndRegisterAutomaticDiagnostic(String robotName, BehaviorDispatcher<HumanoidBehaviorType> dispatcher,
                                                     FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, YoDouble yoTime,
                                                     ROS2Node ros2Node, CapturePointUpdatable capturePointUpdatable,
                                                     WholeBodyControllerParameters wholeBodyControllerParameters,
                                                     FootstepPlannerParametersBasics footstepPlannerParameters, double timeToWait,
                                                     YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoBoolean yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();
      YoEnum<RobotSide> yoSupportLeg = capturePointUpdatable.getYoSupportLeg();
      YoFrameConvexPolygon2D yoSupportPolygon = capturePointUpdatable.getYoSupportPolygon();

      DiagnosticBehavior diagnosticBehavior = new DiagnosticBehavior(robotName,
                                                                     fullRobotModel,
                                                                     yoSupportLeg,
                                                                     referenceFrames,
                                                                     yoTime,
                                                                     yoDoubleSupport,
                                                                     ros2Node,
                                                                     wholeBodyControllerParameters,
                                                                     footstepPlannerParameters,
                                                                     yoSupportPolygon,
                                                                     yoGraphicsListRegistry);
      diagnosticBehavior.setupForAutomaticDiagnostic(timeToWait);
      dispatcher.addBehavior(HumanoidBehaviorType.DIAGNOSTIC, diagnosticBehavior);
      dispatcher.requestBehavior(HumanoidBehaviorType.DIAGNOSTIC);
   }

   public static IHMCHumanoidBehaviorManager createBehaviorModuleForAutomaticDiagnostic(String robotName,
                                                                                        FootstepPlannerParametersBasics footstepPlannerParameters,
                                                                                        WholeBodyControllerParameters wholeBodyControllerParameters,
                                                                                        FullHumanoidRobotModelFactory robotModelFactory,
                                                                                        LogModelProvider modelProvider,
                                                                                        boolean startYoVariableServer,
                                                                                        HumanoidRobotSensorInformation sensorInfo,
                                                                                        double timeToWait)
         throws IOException
   {
      IHMCHumanoidBehaviorManager.setAutomaticDiagnosticTimeToWait(timeToWait);
      IHMCHumanoidBehaviorManager ihmcHumanoidBehaviorManager = new IHMCHumanoidBehaviorManager(robotName,
                                                                                                footstepPlannerParameters,
                                                                                                wholeBodyControllerParameters,
                                                                                                robotModelFactory,
                                                                                                modelProvider,
                                                                                                startYoVariableServer,
                                                                                                sensorInfo,
                                                                                                true,
                                                                                                PubSubImplementation.FAST_RTPS);
      return ihmcHumanoidBehaviorManager;
   }

   public static ROS2Topic getBehaviorRosTopicPrefix(String robotName, String suffix)
   {
      return ROS2Tools.BEHAVIOR_MODULE.withRobot(robotName).withSuffix(suffix);
   }

   public static ROS2Topic getBehaviorOutputRosTopicPrefix(String robotName)
   {
      return getBehaviorRosTopicPrefix(robotName, ROS2Tools.OUTPUT);
   }

   public static ROS2Topic getBehaviorInputRosTopicPrefix(String robotName)
   {
      return getBehaviorRosTopicPrefix(robotName, ROS2Tools.INPUT);
   }

   public static ROS2Topic getOutputTopic(String robotName)
   {
      return ROS2Tools.BEHAVIOR_MODULE.withRobot(robotName).withOutput();
   }

   public static ROS2Topic getInputTopic(String robotName)
   {
      return ROS2Tools.BEHAVIOR_MODULE.withRobot(robotName).withInput();
   }

   @Override
   public void closeAndDispose()
   {
      dispatcher.closeAndDispose();
      ros2Node.destroy();
   }
}
