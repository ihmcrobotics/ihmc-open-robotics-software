package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.NetworkProcessor;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorDisptacher;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorControlModeSubscriber;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorTypeSubscriber;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class BehaviorDispatcherTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   
   private DRCSimulationTestHelper drcSimulationTestHelper;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }
      
      GlobalTimer.clearTimers();
      TimerTaskScheduler.cancelAndReset();
      AsyncContinuousExecutor.cancelAndReset();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   private final double POSITION_THRESHOLD = 0.1;
   private final double ORIENTATION_THRESHOLD = 0.05;

   private static final boolean DEBUG = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();
   private final PacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), PacketDestination.CONTROLLER.ordinal(),
         "DRCControllerCommunicator");
   private final PacketCommunicator networkObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), PacketDestination.NETWORK_PROCESSOR.ordinal(),
         "networkProcessor");
   private final KryoLocalPacketCommunicator behaviorCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
         PacketDestination.BEHAVIOR_MODULE.ordinal(), "behvaiorCommunicator");
   
   private final BehaviorCommunicationBridge communicationBridge = new BehaviorCommunicationBridge(behaviorCommunicator, registry);

   final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final DoubleYoVariable yoTime = new DoubleYoVariable("yoTime", registry);

   private SDFRobot robot;
   private FullRobotModel fullRobotModel;
   private ReferenceFrames referenceFrames;
   private WalkingControllerParameters walkingControllerParameters;

   private RobotDataReceiver robotDataReceiver;
   private BehaviorDisptacher behaviorDispatcher;

   private BooleanYoVariable yoDoubleSupport;

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }
      
      NetworkProcessor networkProcessor = new NetworkProcessor();
      networkProcessor.attachPacketCommunicator(networkObjectCommunicator);
      networkProcessor.attachPacketCommunicator(controllerCommunicator);
      networkProcessor.attachPacketCommunicator(behaviorCommunicator);

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      drcSimulationTestHelper = new DRCSimulationTestHelper(testEnvironment, controllerCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, false, getRobotModel());
      robot = drcSimulationTestHelper.getRobot();
      fullRobotModel = getRobotModel().createFullRobotModel();
      walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      behaviorDispatcher = setupBehaviorDispatcher(fullRobotModel, communicationBridge, yoGraphicsListRegistry);

      CapturePointUpdatable capturePointUpdatable = createCapturePointUpdateable(yoGraphicsListRegistry);
      behaviorDispatcher.addUpdatable(capturePointUpdatable);

      DRCRobotSensorInformation sensorInfo = getRobotModel().getSensorInformation();
      for (RobotSide robotSide : RobotSide.values)
      {
         WristForceSensorFilteredUpdatable wristSensorUpdatable = new WristForceSensorFilteredUpdatable(robotSide, fullRobotModel, sensorInfo,
               robotDataReceiver.getForceSensorDataHolder(), IHMCHumanoidBehaviorManager.BEHAVIOR_YO_VARIABLE_SERVER_DT, controllerCommunicator, registry);
         behaviorDispatcher.addUpdatable(wristSensorUpdatable);
      }

      referenceFrames = robotDataReceiver.getReferenceFrames();
   }

   private CapturePointUpdatable createCapturePointUpdateable(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      controllerCommunicator.attachListener(CapturabilityBasedStatus.class, capturabilityBasedStatusSubsrciber);

      CapturePointUpdatable ret = new CapturePointUpdatable(capturabilityBasedStatusSubsrciber, yoGraphicsListRegistry, registry);

      return ret;
   }

   private BehaviorDisptacher setupBehaviorDispatcher(FullRobotModel fullRobotModel, BehaviorCommunicationBridge communicationBridge,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder);

      HumanoidBehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new HumanoidBehaviorControlModeSubscriber();
      networkObjectCommunicator.attachListener(HumanoidBehaviorControlModePacket.class, desiredBehaviorControlSubscriber);

      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();
      networkObjectCommunicator.attachListener(HumanoidBehaviorTypePacket.class, desiredBehaviorSubscriber);

      YoVariableServer yoVariableServer = null;
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(false);

      BehaviorDisptacher ret = new BehaviorDisptacher(yoTime, robotDataReceiver, desiredBehaviorControlSubscriber, desiredBehaviorSubscriber,
            communicationBridge, yoVariableServer, registry, yoGraphicsListRegistry);

      return ret;
   }

   @AverageDuration
   @Test(timeout = 300000)
   public void testDispatchWalkToLocationBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(communicationBridge, fullRobotModel, referenceFrames,
            walkingControllerParameters);
      behaviorDispatcher.addHumanoidBehavior(HumanoidBehaviorType.WALK_TO_OBJECT, walkToLocationBehavior);

      Thread dispatcherThread = new Thread(behaviorDispatcher, "BehaviorDispatcher");
      dispatcherThread.start();

      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_OBJECT);
      networkObjectCommunicator.send(requestWalkToObjectBehaviorPacket);
      SysoutTool.println("Requesting WalkToLocationBehavior", DEBUG);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = 2.0;
      FramePose2d targetMidFeetPose = offsetCurrentRobotMidFeetZUpPose(walkDistance);
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose);
      SysoutTool.println("Setting WalkToLocationBehavior Target", DEBUG);

      while (!walkToLocationBehavior.isDone() && yoTime.getDoubleValue() < 20.0)
      {
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
         assertTrue(success);
      }

      FramePose2d finalMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);
      assertPosesAreWithinThresholds(targetMidFeetPose, finalMidFeetPose);
      assertTrue(walkToLocationBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration
   @Test(timeout = 300000)
   public void testDispatchWalkToLocationBehaviorAndStop() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(communicationBridge, fullRobotModel, referenceFrames,
            walkingControllerParameters);
      behaviorDispatcher.addHumanoidBehavior(HumanoidBehaviorType.WALK_TO_OBJECT, walkToLocationBehavior);

      Thread dispatcherThread = new Thread(behaviorDispatcher, "BehaviorDispatcher");
      dispatcherThread.start();

      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_OBJECT);
      networkObjectCommunicator.send(requestWalkToObjectBehaviorPacket);
      SysoutTool.println("Requesting WalkToLocationBehavior", DEBUG);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = 2.0;
      FramePose2d targetMidFeetPose = offsetCurrentRobotMidFeetZUpPose(walkDistance);
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose);
      SysoutTool.println("Setting WalkToLocationBehavior Target", DEBUG);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);
      FramePose2d poseBeforeStop = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);

      HumanoidBehaviorControlModePacket stopModePacket = new HumanoidBehaviorControlModePacket(HumanoidBehaviorControlModeEnum.STOP);
      networkObjectCommunicator.send(stopModePacket);
      SysoutTool.println("Sending Stop Request", DEBUG);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);
      FramePose2d poseTwoSecondsAfterStop = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);

      double distanceWalkedAfterStopRequest = poseTwoSecondsAfterStop.getPositionDistance(poseBeforeStop);
      assertTrue(distanceWalkedAfterStopRequest < walkingControllerParameters.getMaxStepLength());
      assertTrue(!walkToLocationBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration
   @Test(timeout = 300000)
   public void testDispatchWalkToLocationBehaviorPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(communicationBridge, fullRobotModel, referenceFrames,
            walkingControllerParameters);
      behaviorDispatcher.addHumanoidBehavior(HumanoidBehaviorType.WALK_TO_OBJECT, walkToLocationBehavior);

      Thread dispatcherThread = new Thread(behaviorDispatcher, "BehaviorDispatcher");
      dispatcherThread.start();

      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_OBJECT);
      networkObjectCommunicator.send(requestWalkToObjectBehaviorPacket);
      SysoutTool.println("Requesting WalkToLocationBehavior", DEBUG);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = 2.0;
      FramePose2d targetMidFeetPose = offsetCurrentRobotMidFeetZUpPose(walkDistance);
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose);
      SysoutTool.println("Setting WalkToLocationBehavior Target", DEBUG);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);
      FramePose2d poseBeforePause = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);

      HumanoidBehaviorControlModePacket pauseModePacket = new HumanoidBehaviorControlModePacket(HumanoidBehaviorControlModeEnum.PAUSE);
      networkObjectCommunicator.send(pauseModePacket);
      SysoutTool.println("Sending Pause Request", DEBUG);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);
      FramePose2d poseTwoSecondsAfterPause = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);

      double distanceWalkedAfterStopRequest = poseTwoSecondsAfterPause.getPositionDistance(poseBeforePause);
      assertTrue(distanceWalkedAfterStopRequest < walkingControllerParameters.getMaxStepLength());
      assertTrue(!walkToLocationBehavior.isDone());

      HumanoidBehaviorControlModePacket resumeModePacket = new HumanoidBehaviorControlModePacket(HumanoidBehaviorControlModeEnum.RESUME);
      networkObjectCommunicator.send(resumeModePacket);
      SysoutTool.println("Sending Resume Request", DEBUG);

      while (!walkToLocationBehavior.isDone() && yoTime.getDoubleValue() < 20.0)
      {
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
         assertTrue(success);
      }

      assertTrue(walkToLocationBehavior.isDone());
      FramePose2d finalMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);
      assertPosesAreWithinThresholds(targetMidFeetPose, finalMidFeetPose);

      BambooTools.reportTestFinishedMessage();
   }

   private FramePose2d offsetCurrentRobotMidFeetZUpPose(double walkDistance)
   {
      FramePose2d targetMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);
      targetMidFeetPose.setX(targetMidFeetPose.getX() + walkDistance);

      return targetMidFeetPose;
   }

   private FramePose2d getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(SDFRobot robot)
   {
      FramePose midFeetPose = getRobotMidFeetPose(robot);

      FramePose2d ret = new FramePose2d();
      ret.setPoseIncludingFrame(ReferenceFrame.getWorldFrame(), midFeetPose.getX(), midFeetPose.getY(), midFeetPose.getYaw());

      return ret;
   }

   private FramePose getRobotMidFeetPose(SDFRobot robot)
   {
      FramePose leftFootPose = getRobotFootPose(robot, RobotSide.LEFT);
      FramePose rightFootPose = getRobotFootPose(robot, RobotSide.RIGHT);

      FramePose ret = new FramePose();
      ret.interpolate(leftFootPose, rightFootPose, 0.5);

      return ret;
   }

   private FramePose getRobotFootPose(SDFRobot robot, RobotSide robotSide)
   {
      List<GroundContactPoint> gcPoints = robot.getFootGroundContactPoints(robotSide);
      Joint ankleJoint = gcPoints.get(0).getParentJoint();
      RigidBodyTransform ankleTransformToWorld = new RigidBodyTransform();
      ankleJoint.getTransformToWorld(ankleTransformToWorld);

      FramePose ret = new FramePose();
      ret.setPose(ankleTransformToWorld);

      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose2d framePose1, FramePose2d framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         SysoutTool.println(" desired Midfeet Pose :\n" + framePose1 + "\n");
         SysoutTool.println(" actual Midfeet Pose :\n" + framePose2 + "\n");

         SysoutTool.println(" positionDistance = " + positionDistance);
         SysoutTool.println(" orientationDistance = " + orientationDistance);
      }

      assertEquals(0.0, positionDistance, POSITION_THRESHOLD);
      assertEquals(0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }
}
