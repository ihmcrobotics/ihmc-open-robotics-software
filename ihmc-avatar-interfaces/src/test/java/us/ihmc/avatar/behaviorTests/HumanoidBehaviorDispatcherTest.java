package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.DiagnosticBehavior;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.DiagnosticBehavior.DiagnosticTask;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisOrientationTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorControlModeSubscriber;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorDispatcher;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorTypeSubscriber;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class HumanoidBehaviorDispatcherTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

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

      behaviorCommunicatorClient.disconnect();
      behaviorCommunicatorServer.disconnect();

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         behaviorDispatcher.closeAndDispose();
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
         communicationBridge = null;
         yoTime = null;
         robot = null;
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

   private ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private CommunicationBridge communicationBridge;
   private YoDouble yoTime;

   private HumanoidFloatingRootJointRobot robot;
   private FullHumanoidRobotModel fullRobotModel;
   private HumanoidReferenceFrames referenceFrames;
   private WalkingControllerParameters walkingControllerParameters;

   private HumanoidRobotDataReceiver robotDataReceiver;
   private BehaviorDispatcher<HumanoidBehaviorType> behaviorDispatcher;

   private PacketCommunicator behaviorCommunicatorServer;

   private PacketCommunicator behaviorCommunicatorClient;

   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private YoVariableRegistry registry;

   @Before
   public void setUp()
   {
      PacketRouter<PacketDestination> networkProcessor = new PacketRouter<>(PacketDestination.class);
      registry = new YoVariableRegistry(getClass().getSimpleName());
      this.yoTime = new YoDouble("yoTime", registry);

      behaviorCommunicatorClient = PacketCommunicator.createIntraprocessPacketCommunicator(
            NetworkPorts.BEHAVIOUR_MODULE_PORT, new IHMCCommunicationKryoNetClassList());

      behaviorCommunicatorServer = PacketCommunicator.createIntraprocessPacketCommunicator(
            NetworkPorts.BEHAVIOUR_MODULE_PORT, new IHMCCommunicationKryoNetClassList());
      try
      {
         behaviorCommunicatorClient.connect();
         behaviorCommunicatorServer.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }


      this.communicationBridge = new CommunicationBridge(behaviorCommunicatorServer);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getSimpleRobotName());

      networkProcessor.attachPacketCommunicator(PacketDestination.CONTROLLER, drcSimulationTestHelper.getControllerCommunicator());
      networkProcessor.attachPacketCommunicator(PacketDestination.BEHAVIOR_MODULE, behaviorCommunicatorClient);

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      robot = drcSimulationTestHelper.getRobot();
      fullRobotModel = getRobotModel().createFullRobotModel();
      walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      behaviorDispatcher = setupBehaviorDispatcher(fullRobotModel, communicationBridge, yoGraphicsListRegistry, behaviorCommunicatorServer, registry);

      CapturePointUpdatable capturePointUpdatable = createCapturePointUpdateable(drcSimulationTestHelper, registry, yoGraphicsListRegistry);
      behaviorDispatcher.addUpdatable(capturePointUpdatable);

      DRCRobotSensorInformation sensorInfo = getRobotModel().getSensorInformation();
      for (RobotSide robotSide : RobotSide.values)
      {
         WristForceSensorFilteredUpdatable wristSensorUpdatable = new WristForceSensorFilteredUpdatable(robotSide, fullRobotModel, sensorInfo,
               robotDataReceiver.getForceSensorDataHolder(), IHMCHumanoidBehaviorManager.BEHAVIOR_YO_VARIABLE_SERVER_DT, drcSimulationTestHelper.getControllerCommunicator(), registry);
         behaviorDispatcher.addUpdatable(wristSensorUpdatable);
      }

      referenceFrames = robotDataReceiver.getReferenceFrames();
   }

   private CapturePointUpdatable createCapturePointUpdateable(DRCSimulationTestHelper testHelper,
         YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      testHelper.attachListener(CapturabilityBasedStatus.class, capturabilityBasedStatusSubsrciber);

      CapturePointUpdatable ret = new CapturePointUpdatable(capturabilityBasedStatusSubsrciber, yoGraphicsListRegistry, registry);

      return ret;
   }

   private BehaviorDispatcher<HumanoidBehaviorType> setupBehaviorDispatcher(FullHumanoidRobotModel fullRobotModel, CommunicationBridge communicationBridge,
         YoGraphicsListRegistry yoGraphicsListRegistry, PacketCommunicator behaviorCommunicatorServer, YoVariableRegistry registry)
   {
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      robotDataReceiver = new HumanoidRobotDataReceiver(fullRobotModel, forceSensorDataHolder);
      behaviorCommunicatorServer.attachListener(RobotConfigurationData.class, robotDataReceiver);

      BehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new BehaviorControlModeSubscriber();
      behaviorCommunicatorServer.attachListener(BehaviorControlModePacket.class, desiredBehaviorControlSubscriber);

      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();
      behaviorCommunicatorServer.attachListener(HumanoidBehaviorTypePacket.class, desiredBehaviorSubscriber);

      YoVariableServer yoVariableServer = null;
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(false);

      BehaviorDispatcher<HumanoidBehaviorType> ret = new BehaviorDispatcher<>(yoTime, robotDataReceiver, desiredBehaviorControlSubscriber, desiredBehaviorSubscriber,
            communicationBridge, yoVariableServer, HumanoidBehaviorType.class, HumanoidBehaviorType.STOP, registry, yoGraphicsListRegistry);

      return ret;
   }

   @ContinuousIntegrationTest(estimatedDuration = 50.0)
   @Test(timeout = 600000)
   public void testDispatchPelvisPoseBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      PelvisOrientationTrajectoryBehavior pelvisOrientationTrajectoryBehavior = new PelvisOrientationTrajectoryBehavior(communicationBridge, yoTime);
      behaviorDispatcher.addBehavior(HumanoidBehaviorType.TEST, pelvisOrientationTrajectoryBehavior);

      behaviorDispatcher.start();

      HumanoidBehaviorTypePacket requestPelvisPoseBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.TEST);
      behaviorCommunicatorClient.send(requestPelvisPoseBehaviorPacket);
      PrintTools.debug(this, "Requesting PelvisPoseBehavior");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      PelvisOrientationTrajectoryMessage pelvisPosePacket = createPelvisOrientationTrajectoryMessage(new Vector3D(0.0, 1.0, 0.0), Math.toRadians(5.0));
      FramePose3D desiredPelvisPose = new FramePose3D();
      desiredPelvisPose.setOrientation(pelvisPosePacket.getSO3Trajectory().getLastTrajectoryPoint().orientation);

      pelvisOrientationTrajectoryBehavior.initialize();
      pelvisOrientationTrajectoryBehavior.setInput(pelvisPosePacket);
      assertTrue(pelvisOrientationTrajectoryBehavior.hasInputBeenSet());
      PrintTools.debug(this, "Setting PelvisPoseBehavior Input");

      PrintTools.debug(this, "Starting to Excecute Behavior");
      while (!pelvisOrientationTrajectoryBehavior.isDone() && yoTime.getDoubleValue() < 20.0)
      {
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
         assertTrue(success);
      }

      assertTrue(pelvisOrientationTrajectoryBehavior.isDone());
      assertOrientationsAreWithinThresholds(desiredPelvisPose, getCurrentPelvisPose());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 600000)
   public void testDispatchWalkToLocationBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(communicationBridge, fullRobotModel, referenceFrames,
            walkingControllerParameters);
      behaviorDispatcher.addBehavior(HumanoidBehaviorType.WALK_TO_LOCATION, walkToLocationBehavior);

      behaviorDispatcher.start();


      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_LOCATION);
      behaviorCommunicatorClient.send(requestWalkToObjectBehaviorPacket);
      PrintTools.debug(this, "Requesting WalkToLocationBehavior");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = 2.0;
      FramePose2D targetMidFeetPose = offsetCurrentRobotMidFeetZUpPose(walkDistance);
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose);
      assertTrue(walkToLocationBehavior.hasInputBeenSet());
      PrintTools.debug(this, "Setting WalkToLocationBehavior Target");

      if (DEBUG)
      {
         ArrayList<Footstep> footsteps = walkToLocationBehavior.getFootSteps();
         Point3D footstepPositionInWorld = new Point3D();
         for (Footstep footStep : footsteps)
         {
            FramePose3D footstepPose = new FramePose3D();
            footStep.getPose(footstepPose);
            footstepPose.changeFrame(ReferenceFrame.getWorldFrame());
            PrintTools.debug(this, footstepPose.getPosition().toString());
         }
      }

      PrintTools.debug(this, "Starting to Excecute Behavior");
      while (!walkToLocationBehavior.isDone() && yoTime.getDoubleValue() < 20.0)
      {
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
         assertTrue(success);
      }

      FramePose2D finalMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);
      assertPosesAreWithinThresholds(targetMidFeetPose, finalMidFeetPose);
      assertTrue(walkToLocationBehavior.isDone());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 600000)
   public void testDispatchKarateKidDiagnosticBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      YoEnum<RobotSide> supportLeg = new YoEnum<>("supportLeg", registry, RobotSide.class);
      supportLeg.set(RobotSide.LEFT);

      YoFrameConvexPolygon2d yoSupportPolygon = new YoFrameConvexPolygon2d("supportPolygon", "", ReferenceFrame.getWorldFrame(), 10, registry);

      WholeBodyControllerParameters wholeBodyControllerParameters = this.getRobotModel();

      YoBoolean yoDoubleSupport = new YoBoolean("doubleSupport", registry);

      DiagnosticBehavior diagnosticBehavior = new DiagnosticBehavior(fullRobotModel, supportLeg, referenceFrames, yoTime, yoDoubleSupport, communicationBridge,
            wholeBodyControllerParameters, yoSupportPolygon, yoGraphicsListRegistry);


      behaviorDispatcher.addBehavior(HumanoidBehaviorType.DIAGNOSTIC, diagnosticBehavior);

      behaviorDispatcher.start();

      HumanoidBehaviorTypePacket requestDiagnosticBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.DIAGNOSTIC);
      behaviorCommunicatorClient.send(requestDiagnosticBehaviorPacket);
      PrintTools.debug(this, "Requesting DiagnosticBehavior");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      diagnosticBehavior.requestDiagnosticBehavior(DiagnosticTask.KARATE_KID);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      assertFalse(diagnosticBehavior.isDone());

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);
      assertTrue(success);

      assertFalse(diagnosticBehavior.isDone());

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(32.0);
      assertTrue(success);

      assertTrue(diagnosticBehavior.isDone());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 600000)
   public void testDispatchWalkToLocationBehaviorAndStop() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(communicationBridge, fullRobotModel, referenceFrames,
            walkingControllerParameters);
      behaviorDispatcher.addBehavior(HumanoidBehaviorType.WALK_TO_LOCATION, walkToLocationBehavior);

      behaviorDispatcher.start();


      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_LOCATION);
      behaviorCommunicatorClient.send(requestWalkToObjectBehaviorPacket);
      PrintTools.debug(this, "Requesting WalkToLocationBehavior");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = 2.0;
      FramePose2D targetMidFeetPose = offsetCurrentRobotMidFeetZUpPose(walkDistance);
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose);
      assertTrue(walkToLocationBehavior.hasInputBeenSet());
      PrintTools.debug(this, "Setting WalkToLocationBehavior Target");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);
      FramePose2D poseBeforeStop = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);

      BehaviorControlModePacket stopModePacket = new BehaviorControlModePacket(BehaviorControlModeEnum.STOP);
      behaviorCommunicatorClient.send(stopModePacket);
      PrintTools.debug(this, "Sending Stop Request");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);
      FramePose2D poseTwoSecondsAfterStop = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);

      double distanceWalkedAfterStopRequest = poseTwoSecondsAfterStop.getPositionDistance(poseBeforeStop);
      assertTrue(distanceWalkedAfterStopRequest < walkingControllerParameters.getSteppingParameters().getMaxStepLength());
      assertTrue(!walkToLocationBehavior.isDone());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 600000)
   public void testDispatchWalkToLocationBehaviorPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(communicationBridge, fullRobotModel, referenceFrames,
            walkingControllerParameters);
      behaviorDispatcher.addBehavior(HumanoidBehaviorType.WALK_TO_LOCATION, walkToLocationBehavior);

      behaviorDispatcher.start();


      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_LOCATION);
      behaviorCommunicatorClient.send(requestWalkToObjectBehaviorPacket);
      PrintTools.debug(this, "Requesting WalkToLocationBehavior");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = 2.0;
      FramePose2D targetMidFeetPose = offsetCurrentRobotMidFeetZUpPose(walkDistance);
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose);
      assertTrue(walkToLocationBehavior.hasInputBeenSet());
      PrintTools.debug(this, "Setting WalkToLocationBehavior Target");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);
      FramePose2D poseBeforePause = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);

      BehaviorControlModePacket pauseModePacket = new BehaviorControlModePacket(BehaviorControlModeEnum.PAUSE);
      behaviorCommunicatorClient.send(pauseModePacket);
      PrintTools.debug(this, "Sending Pause Request");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);
      FramePose2D poseTwoSecondsAfterPause = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);

      double distanceWalkedAfterStopRequest = poseTwoSecondsAfterPause.getPositionDistance(poseBeforePause);
      assertTrue(distanceWalkedAfterStopRequest < walkingControllerParameters.getSteppingParameters().getMaxStepLength());
      assertTrue(!walkToLocationBehavior.isDone());

      BehaviorControlModePacket resumeModePacket = new BehaviorControlModePacket(BehaviorControlModeEnum.RESUME);
      behaviorCommunicatorClient.send(resumeModePacket);
      PrintTools.debug(this, "Sending Resume Request");

      while (!walkToLocationBehavior.isDone() && yoTime.getDoubleValue() < 20.0)
      {
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
         assertTrue(success);
      }

      assertTrue(walkToLocationBehavior.isDone());
      FramePose2D finalMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);
      assertPosesAreWithinThresholds(targetMidFeetPose, finalMidFeetPose);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private FramePose2D offsetCurrentRobotMidFeetZUpPose(double walkDistance)
   {
      FramePose2D targetMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);
      targetMidFeetPose.setX(targetMidFeetPose.getX() + walkDistance);

      return targetMidFeetPose;
   }

   private FramePose2D getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(HumanoidFloatingRootJointRobot robot)
   {
      FramePose3D midFeetPose = getRobotMidFeetPose(robot);

      FramePose2D ret = new FramePose2D();
      ret.setIncludingFrame(ReferenceFrame.getWorldFrame(), midFeetPose.getX(), midFeetPose.getY(), midFeetPose.getYaw());

      return ret;
   }

   private FramePose3D getRobotMidFeetPose(HumanoidFloatingRootJointRobot robot)
   {
      FramePose3D leftFootPose = getRobotFootPose(robot, RobotSide.LEFT);
      FramePose3D rightFootPose = getRobotFootPose(robot, RobotSide.RIGHT);

      FramePose3D ret = new FramePose3D();
      ret.interpolate(leftFootPose, rightFootPose, 0.5);

      return ret;
   }

   private FramePose3D getRobotFootPose(HumanoidFloatingRootJointRobot robot, RobotSide robotSide)
   {
      List<GroundContactPoint> gcPoints = robot.getFootGroundContactPoints(robotSide);
      Joint ankleJoint = gcPoints.get(0).getParentJoint();
      RigidBodyTransform ankleTransformToWorld = new RigidBodyTransform();
      ankleJoint.getTransformToWorld(ankleTransformToWorld);

      FramePose3D ret = new FramePose3D();
      ret.set(ankleTransformToWorld);

      return ret;
   }

   private PelvisOrientationTrajectoryMessage createPelvisOrientationTrajectoryMessage(Vector3D rotationAxis, double rotationAngle)
   {
      AxisAngle desiredPelvisAxisAngle = new AxisAngle(rotationAxis, rotationAngle);
      Quaternion desiredPelvisQuat = new Quaternion();
      desiredPelvisQuat.set(desiredPelvisAxisAngle);

      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(3.0, desiredPelvisQuat);
      return pelvisOrientationTrajectoryMessage;
   }

   private FramePose3D getCurrentPelvisPose()
   {
      fullRobotModel.updateFrames();
      FramePose3D ret = new FramePose3D();
      ret.setToZero(fullRobotModel.getPelvis().getBodyFixedFrame());
      ret.changeFrame(ReferenceFrame.getWorldFrame());

      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose2D framePose1, FramePose2D framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         PrintTools.debug(this, " desired Midfeet Pose :\n" + framePose1 + "\n");
         PrintTools.debug(this, " actual Midfeet Pose :\n" + framePose2 + "\n");

         PrintTools.debug(this, " positionDistance = " + positionDistance);
         PrintTools.debug(this, " orientationDistance = " + orientationDistance);
      }

      assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + POSITION_THRESHOLD, 0.0, positionDistance, POSITION_THRESHOLD);
      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + ORIENTATION_THRESHOLD, 0.0, orientationDistance,
            ORIENTATION_THRESHOLD);
   }

   private void assertOrientationsAreWithinThresholds(FramePose3D framePose1, FramePose3D framePose2)
   {
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         PrintTools.debug(this, " desired Pelvis Pose :\n" + framePose1 + "\n");
         PrintTools.debug(this, " actual Pelvis Pose :\n" + framePose2 + "\n");

         PrintTools.debug(this, " orientationDistance = " + orientationDistance);
      }

      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + ORIENTATION_THRESHOLD, 0.0, orientationDistance,
            ORIENTATION_THRESHOLD);
   }
}
