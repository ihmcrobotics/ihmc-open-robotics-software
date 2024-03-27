package us.ihmc.avatar.behaviorTests;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import toolbox_msgs.msg.dds.BehaviorControlModePacket;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import toolbox_msgs.msg.dds.HumanoidBehaviorTypePacket;
import controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.DiagnosticBehavior;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.DiagnosticBehavior.DiagnosticTask;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisOrientationTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorControlModeSubscriber;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorDispatcher;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorTypeSubscriber;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class HumanoidBehaviorDispatcherTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         behaviorDispatcher.closeAndDispose();
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
         ros2Node = null;
         yoTime = null;
         fullRobotModel = null;
         referenceFrames = null;
         walkingControllerParameters = null;
         robotDataReceiver = null;
         behaviorDispatcher = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private final double POSITION_THRESHOLD = 0.1;
   private final double ORIENTATION_THRESHOLD = 0.05;

   private final boolean DEBUG = false;

   private ROS2Node ros2Node;
   private YoDouble yoTime;

   private FullHumanoidRobotModel fullRobotModel;
   private HumanoidReferenceFrames referenceFrames;
   private WalkingControllerParameters walkingControllerParameters;

   private HumanoidRobotDataReceiver robotDataReceiver;
   private BehaviorDispatcher<HumanoidBehaviorType> behaviorDispatcher;

   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private YoRegistry registry;

   @BeforeEach
   public void setUp()
   {
      registry = new YoRegistry(getClass().getSimpleName());
      this.yoTime = new YoDouble("yoTime", registry);

      this.ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.INTRAPROCESS, "ihmc_humanoid_behavior_dispatcher_test");

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), simulationTestingParameters);
      simulationTestHelper.start();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      fullRobotModel = getRobotModel().createFullRobotModel();
      walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      behaviorDispatcher = setupBehaviorDispatcher(getRobotModel().getSimpleRobotName(), fullRobotModel, ros2Node, yoGraphicsListRegistry, registry);

      CapturePointUpdatable capturePointUpdatable = createCapturePointUpdateable(simulationTestHelper, registry, yoGraphicsListRegistry);
      behaviorDispatcher.addUpdatable(capturePointUpdatable);

      HumanoidRobotSensorInformation sensorInfo = getRobotModel().getSensorInformation();
      for (RobotSide robotSide : RobotSide.values)
      {
         WristForceSensorFilteredUpdatable wristSensorUpdatable = new WristForceSensorFilteredUpdatable(getSimpleRobotName(),
                                                                                                        robotSide,
                                                                                                        fullRobotModel,
                                                                                                        sensorInfo,
                                                                                                        robotDataReceiver.getForceSensorDataHolder(),
                                                                                                        IHMCHumanoidBehaviorManager.BEHAVIOR_YO_VARIABLE_SERVER_DT,
                                                                                                        simulationTestHelper.getROS2Node(),
                                                                                                        registry);
         behaviorDispatcher.addUpdatable(wristSensorUpdatable);
      }

      referenceFrames = robotDataReceiver.getReferenceFrames();
   }

   private CapturePointUpdatable createCapturePointUpdateable(SCS2AvatarTestingSimulation testHelper,
                                                              YoRegistry registry,
                                                              YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      testHelper.createSubscriberFromController(CapturabilityBasedStatus.class, capturabilityBasedStatusSubsrciber::receivedPacket);

      return new CapturePointUpdatable(capturabilityBasedStatusSubsrciber, yoGraphicsListRegistry, registry);
   }

   private BehaviorDispatcher<HumanoidBehaviorType> setupBehaviorDispatcher(String robotName,
                                                                            FullHumanoidRobotModel fullRobotModel,
                                                                            ROS2Node ros2Node,
                                                                            YoGraphicsListRegistry yoGraphicsListRegistry,
                                                                            YoRegistry registry)
   {
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      robotDataReceiver = new HumanoidRobotDataReceiver(fullRobotModel, forceSensorDataHolder);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class, ROS2Tools.getControllerOutputTopic(robotName), s ->
      {
         if (robotDataReceiver != null && s != null)
            robotDataReceiver.receivedPacket(s.takeNextData());
      });

      BehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new BehaviorControlModeSubscriber();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    BehaviorControlModePacket.class,
                                                    IHMCHumanoidBehaviorManager.getInputTopic(robotName),
                                                    s -> desiredBehaviorControlSubscriber.receivedPacket(s.takeNextData()));

      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    HumanoidBehaviorTypePacket.class,
                                                    IHMCHumanoidBehaviorManager.getInputTopic(robotName),
                                                    s -> desiredBehaviorSubscriber.receivedPacket(s.takeNextData()));

      YoVariableServer yoVariableServer = null;
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(false);

      BehaviorDispatcher<HumanoidBehaviorType> ret = new BehaviorDispatcher<>(robotName,
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

      return ret;
   }

   @Test
   public void testDispatchPelvisPoseBehavior()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      PelvisOrientationTrajectoryBehavior pelvisOrientationTrajectoryBehavior = new PelvisOrientationTrajectoryBehavior(getSimpleRobotName(), ros2Node, yoTime);
      behaviorDispatcher.addBehavior(HumanoidBehaviorType.TEST, pelvisOrientationTrajectoryBehavior);

      behaviorDispatcher.start();

      HumanoidBehaviorTypePacket requestPelvisPoseBehaviorPacket = HumanoidMessageTools.createHumanoidBehaviorTypePacket(HumanoidBehaviorType.TEST);
      simulationTestHelper.createPublisher(HumanoidBehaviorTypePacket.class, IHMCHumanoidBehaviorManager.getInputTopic(getSimpleRobotName()))
                          .publish(requestPelvisPoseBehaviorPacket);
      LogTools.debug(this, "Requesting PelvisPoseBehavior");

      success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      PelvisOrientationTrajectoryMessage pelvisPosePacket = createPelvisOrientationTrajectoryMessage(new Vector3D(0.0, 1.0, 0.0), Math.toRadians(5.0));
      FramePose3D desiredPelvisPose = new FramePose3D();
      desiredPelvisPose.getOrientation().set(pelvisPosePacket.getSo3Trajectory().getTaskspaceTrajectoryPoints().getLast().getOrientation());

      pelvisOrientationTrajectoryBehavior.initialize();
      pelvisOrientationTrajectoryBehavior.setInput(pelvisPosePacket);
      assertTrue(pelvisOrientationTrajectoryBehavior.hasInputBeenSet());
      LogTools.debug(this, "Setting PelvisPoseBehavior Input");

      LogTools.debug(this, "Starting to Excecute Behavior");
      while (!pelvisOrientationTrajectoryBehavior.isDone() && yoTime.getDoubleValue() < 20.0)
      {
         success = simulationTestHelper.simulateNow(4.0);
         assertTrue(success);
      }

      assertTrue(pelvisOrientationTrajectoryBehavior.isDone());
      assertOrientationsAreWithinThresholds(desiredPelvisPose, getCurrentPelvisPose());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testDispatchWalkToLocationBehavior()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(getSimpleRobotName(),
                                                                                 ros2Node,
                                                                                 fullRobotModel,
                                                                                 referenceFrames,
                                                                                 walkingControllerParameters);
      behaviorDispatcher.addBehavior(HumanoidBehaviorType.WALK_TO_LOCATION, walkToLocationBehavior);

      behaviorDispatcher.start();

      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = HumanoidMessageTools.createHumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_LOCATION);
      simulationTestHelper.createPublisher(HumanoidBehaviorTypePacket.class, IHMCHumanoidBehaviorManager.getInputTopic(getSimpleRobotName()))
                          .publish(requestWalkToObjectBehaviorPacket);
      LogTools.debug(this, "Requesting WalkToLocationBehavior");

      success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      double walkDistance = 2.0;
      FramePose2D targetMidFeetPose = offsetCurrentRobotMidFeetZUpPose(walkDistance);
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose);
      assertTrue(walkToLocationBehavior.hasInputBeenSet());
      LogTools.debug(this, "Setting WalkToLocationBehavior Target");

      if (DEBUG)
      {
         ArrayList<Footstep> footsteps = walkToLocationBehavior.getFootSteps();
         Point3D footstepPositionInWorld = new Point3D();
         for (Footstep footStep : footsteps)
         {
            FramePose3D footstepPose = new FramePose3D();
            footStep.getPose(footstepPose);
            footstepPose.changeFrame(worldFrame);
            LogTools.debug(this, footstepPose.getPosition().toString());
         }
      }

      LogTools.debug(this, "Starting to Excecute Behavior");
      while (!walkToLocationBehavior.isDone() && yoTime.getDoubleValue() < 20.0)
      {
         success = simulationTestHelper.simulateNow(4.0);
         assertTrue(success);
      }

      FramePose2D finalMidFeetPose = new FramePose2D(simulationTestHelper.getControllerReferenceFrames().getMidFeetZUpFrame());
      finalMidFeetPose.changeFrame(worldFrame);
      assertPosesAreWithinThresholds(targetMidFeetPose, finalMidFeetPose);
      assertTrue(walkToLocationBehavior.isDone());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testDispatchKarateKidDiagnosticBehavior()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      YoEnum<RobotSide> supportLeg = new YoEnum<>("supportLeg", registry, RobotSide.class);
      supportLeg.set(RobotSide.LEFT);

      YoFrameConvexPolygon2D yoSupportPolygon = new YoFrameConvexPolygon2D("supportPolygon", "", worldFrame, 10, registry);

      WholeBodyControllerParameters<RobotSide> wholeBodyControllerParameters = this.getRobotModel();

      YoBoolean yoDoubleSupport = new YoBoolean("doubleSupport", registry);

      DiagnosticBehavior diagnosticBehavior = new DiagnosticBehavior(getSimpleRobotName(),
                                                                     fullRobotModel,
                                                                     supportLeg,
                                                                     referenceFrames,
                                                                     yoTime,
                                                                     yoDoubleSupport,
                                                                     ros2Node,
                                                                     wholeBodyControllerParameters,
                                                                     getRobotModel().getFootstepPlannerParameters(),
                                                                     yoSupportPolygon,
                                                                     yoGraphicsListRegistry);

      behaviorDispatcher.addBehavior(HumanoidBehaviorType.DIAGNOSTIC, diagnosticBehavior);

      behaviorDispatcher.start();

      HumanoidBehaviorTypePacket requestDiagnosticBehaviorPacket = HumanoidMessageTools.createHumanoidBehaviorTypePacket(HumanoidBehaviorType.DIAGNOSTIC);
      simulationTestHelper.createPublisher(HumanoidBehaviorTypePacket.class, IHMCHumanoidBehaviorManager.getInputTopic(getSimpleRobotName()))
                          .publish(requestDiagnosticBehaviorPacket);
      LogTools.debug(this, "Requesting DiagnosticBehavior");

      success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      diagnosticBehavior.requestDiagnosticBehavior(DiagnosticTask.KARATE_KID);

      success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      assertFalse(diagnosticBehavior.isDone());

      success = simulationTestHelper.simulateNow(10.0);
      assertTrue(success);

      assertFalse(diagnosticBehavior.isDone());

      success = simulationTestHelper.simulateNow(36.0);
      assertTrue(success);

      assertTrue(diagnosticBehavior.isDone());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testDispatchWalkToLocationBehaviorAndStop()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(getSimpleRobotName(),
                                                                                 ros2Node,
                                                                                 fullRobotModel,
                                                                                 referenceFrames,
                                                                                 walkingControllerParameters);
      behaviorDispatcher.addBehavior(HumanoidBehaviorType.WALK_TO_LOCATION, walkToLocationBehavior);

      behaviorDispatcher.start();

      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = HumanoidMessageTools.createHumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_LOCATION);
      simulationTestHelper.createPublisher(HumanoidBehaviorTypePacket.class, IHMCHumanoidBehaviorManager.getInputTopic(getSimpleRobotName()))
                          .publish(requestWalkToObjectBehaviorPacket);
      LogTools.debug(this, "Requesting WalkToLocationBehavior");

      success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      double walkDistance = 2.0;
      FramePose2D targetMidFeetPose = offsetCurrentRobotMidFeetZUpPose(walkDistance);
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose);
      assertTrue(walkToLocationBehavior.hasInputBeenSet());
      LogTools.debug(this, "Setting WalkToLocationBehavior Target");

      success = simulationTestHelper.simulateNow(2.0);
      assertTrue(success);
      FramePose2D poseBeforeStop = new FramePose2D(simulationTestHelper.getControllerReferenceFrames().getMidFeetZUpFrame());
      targetMidFeetPose.changeFrame(worldFrame);

      BehaviorControlModePacket stopModePacket = HumanoidMessageTools.createBehaviorControlModePacket(BehaviorControlModeEnum.STOP);
      simulationTestHelper.createPublisher(BehaviorControlModePacket.class, IHMCHumanoidBehaviorManager.getInputTopic(getSimpleRobotName()))
                          .publish(stopModePacket);
      LogTools.debug(this, "Sending Stop Request");

      success = simulationTestHelper.simulateNow(2.0);
      assertTrue(success);
      FramePose2D poseTwoSecondsAfterStop = new FramePose2D(simulationTestHelper.getControllerReferenceFrames().getMidFeetZUpFrame());
      targetMidFeetPose.changeFrame(worldFrame);

      double distanceWalkedAfterStopRequest = poseTwoSecondsAfterStop.getPositionDistance(poseBeforeStop);
      assertTrue(distanceWalkedAfterStopRequest < walkingControllerParameters.getSteppingParameters().getMaxStepLength());
      assertTrue(!walkToLocationBehavior.isDone());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testDispatchWalkToLocationBehaviorPauseAndResume()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(getSimpleRobotName(),
                                                                                 ros2Node,
                                                                                 fullRobotModel,
                                                                                 referenceFrames,
                                                                                 walkingControllerParameters);
      behaviorDispatcher.addBehavior(HumanoidBehaviorType.WALK_TO_LOCATION, walkToLocationBehavior);

      behaviorDispatcher.start();

      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = HumanoidMessageTools.createHumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_LOCATION);
      simulationTestHelper.createPublisher(HumanoidBehaviorTypePacket.class, IHMCHumanoidBehaviorManager.getInputTopic(getSimpleRobotName()))
                          .publish(requestWalkToObjectBehaviorPacket);
      LogTools.debug(this, "Requesting WalkToLocationBehavior");

      success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      double walkDistance = 2.0;
      FramePose2D targetMidFeetPose = offsetCurrentRobotMidFeetZUpPose(walkDistance);
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose);
      assertTrue(walkToLocationBehavior.hasInputBeenSet());
      LogTools.debug(this, "Setting WalkToLocationBehavior Target");

      success = simulationTestHelper.simulateNow(2.0);
      assertTrue(success);
      FramePose2D poseBeforePause = new FramePose2D(simulationTestHelper.getControllerReferenceFrames().getMidFeetZUpFrame());
      targetMidFeetPose.changeFrame(worldFrame);

      BehaviorControlModePacket pauseModePacket = HumanoidMessageTools.createBehaviorControlModePacket(BehaviorControlModeEnum.PAUSE);
      ROS2PublisherBasics<BehaviorControlModePacket> publisher = simulationTestHelper.createPublisher(BehaviorControlModePacket.class,
                                                                                                    IHMCHumanoidBehaviorManager.getInputTopic(getSimpleRobotName()));
      publisher.publish(pauseModePacket);
      LogTools.debug(this, "Sending Pause Request");

      success = simulationTestHelper.simulateNow(2.0);
      assertTrue(success);
      FramePose2D poseTwoSecondsAfterPause = new FramePose2D(simulationTestHelper.getControllerReferenceFrames().getMidFeetZUpFrame());
      targetMidFeetPose.changeFrame(worldFrame);

      double distanceWalkedAfterStopRequest = poseTwoSecondsAfterPause.getPositionDistance(poseBeforePause);
      assertTrue(distanceWalkedAfterStopRequest < walkingControllerParameters.getSteppingParameters().getMaxStepLength());
      assertTrue(!walkToLocationBehavior.isDone());

      BehaviorControlModePacket resumeModePacket = HumanoidMessageTools.createBehaviorControlModePacket(BehaviorControlModeEnum.RESUME);
      publisher.publish(resumeModePacket);
      LogTools.debug(this, "Sending Resume Request");

      while (!walkToLocationBehavior.isDone() && yoTime.getDoubleValue() < 20.0)
      {
         success = simulationTestHelper.simulateNow(4.0);
         assertTrue(success);
      }

      assertTrue(walkToLocationBehavior.isDone());
      FramePose2D finalMidFeetPose = new FramePose2D(simulationTestHelper.getControllerReferenceFrames().getMidFeetZUpFrame());
      targetMidFeetPose.changeFrame(worldFrame);
      assertPosesAreWithinThresholds(targetMidFeetPose, finalMidFeetPose);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private FramePose2D offsetCurrentRobotMidFeetZUpPose(double walkDistance)
   {
      FramePose2D targetMidFeetPose = new FramePose2D(simulationTestHelper.getControllerReferenceFrames().getMidFeetZUpFrame());
      targetMidFeetPose.changeFrame(worldFrame);
      targetMidFeetPose.setX(targetMidFeetPose.getX() + walkDistance);
      return targetMidFeetPose;
   }

   private PelvisOrientationTrajectoryMessage createPelvisOrientationTrajectoryMessage(Vector3D rotationAxis, double rotationAngle)
   {
      AxisAngle desiredPelvisAxisAngle = new AxisAngle(rotationAxis, rotationAngle);
      Quaternion desiredPelvisQuat = new Quaternion();
      desiredPelvisQuat.set(desiredPelvisAxisAngle);

      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = HumanoidMessageTools.createPelvisOrientationTrajectoryMessage(3.0,
                                                                                                                                            desiredPelvisQuat);
      return pelvisOrientationTrajectoryMessage;
   }

   private FramePose3D getCurrentPelvisPose()
   {
      fullRobotModel.updateFrames();
      FramePose3D ret = new FramePose3D();
      ret.setToZero(fullRobotModel.getPelvis().getBodyFixedFrame());
      ret.changeFrame(worldFrame);
      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose2D framePose1, FramePose2D framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         LogTools.debug(this, " desired Midfeet Pose :\n" + framePose1 + "\n");
         LogTools.debug(this, " actual Midfeet Pose :\n" + framePose2 + "\n");

         LogTools.debug(this, " positionDistance = " + positionDistance);
         LogTools.debug(this, " orientationDistance = " + orientationDistance);
      }

      assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + POSITION_THRESHOLD, 0.0, positionDistance, POSITION_THRESHOLD);
      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + ORIENTATION_THRESHOLD,
                   0.0,
                   orientationDistance,
                   ORIENTATION_THRESHOLD);
   }

   private void assertOrientationsAreWithinThresholds(FramePose3D framePose1, FramePose3D framePose2)
   {
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         LogTools.debug(this, " desired Pelvis Pose :\n" + framePose1 + "\n");
         LogTools.debug(this, " actual Pelvis Pose :\n" + framePose2 + "\n");

         LogTools.debug(this, " orientationDistance = " + orientationDistance);
      }

      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + ORIENTATION_THRESHOLD,
                   0.0,
                   orientationDistance,
                   ORIENTATION_THRESHOLD);
   }
}
