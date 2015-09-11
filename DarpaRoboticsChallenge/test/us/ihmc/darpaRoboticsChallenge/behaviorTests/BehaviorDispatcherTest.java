package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorDisptacher;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorControlModeSubscriber;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorTypeSubscriber;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

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
      
      behaviorCommunicatorClient.close();
      behaviorCommunicatorServer.close();

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
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
         yoDoubleSupport = null;
      }

      GlobalTimer.clearTimers();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private final double POSITION_THRESHOLD = 0.1;
   private final double ORIENTATION_THRESHOLD = 0.05;

   private final boolean DEBUG = false;

   private ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private BehaviorCommunicationBridge communicationBridge;
   private DoubleYoVariable yoTime;

   private SDFHumanoidRobot robot;
   private FullHumanoidRobotModel fullRobotModel;
   private HumanoidReferenceFrames referenceFrames;
   private WalkingControllerParameters walkingControllerParameters;

   private RobotDataReceiver robotDataReceiver;
   private BehaviorDisptacher behaviorDispatcher;

   private BooleanYoVariable yoDoubleSupport;

   private PacketCommunicator behaviorCommunicatorServer;

   private PacketCommunicator behaviorCommunicatorClient;

   @Before
   public void setUp()
   {
      PacketRouter<PacketDestination> networkProcessor = new PacketRouter<>(PacketDestination.class);
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      this.yoTime = new DoubleYoVariable("yoTime", registry);

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
      
      
      this.communicationBridge = new BehaviorCommunicationBridge(behaviorCommunicatorServer, registry);
      
      drcSimulationTestHelper = new DRCSimulationTestHelper(new DRCDemo01NavigationEnvironment(), getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());

      networkProcessor.attachPacketCommunicator(PacketDestination.CONTROLLER, drcSimulationTestHelper.getControllerCommunicator());
      networkProcessor.attachPacketCommunicator(PacketDestination.BEHAVIOR_MODULE, behaviorCommunicatorClient);

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      robot = drcSimulationTestHelper.getRobot();
      fullRobotModel = getRobotModel().createFullRobotModel();
      walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

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

   private BehaviorDisptacher setupBehaviorDispatcher(FullHumanoidRobotModel fullRobotModel, BehaviorCommunicationBridge communicationBridge,
         YoGraphicsListRegistry yoGraphicsListRegistry, PacketCommunicator behaviorCommunicatorServer, YoVariableRegistry registry)
   {
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder);
      behaviorCommunicatorServer.attachListener(RobotConfigurationData.class, robotDataReceiver);

      HumanoidBehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new HumanoidBehaviorControlModeSubscriber();
      behaviorCommunicatorServer.attachListener(HumanoidBehaviorControlModePacket.class, desiredBehaviorControlSubscriber);

      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();
      behaviorCommunicatorServer.attachListener(HumanoidBehaviorTypePacket.class, desiredBehaviorSubscriber);

      YoVariableServer yoVariableServer = null;
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(false);

      BehaviorDisptacher ret = new BehaviorDisptacher(yoTime, robotDataReceiver, desiredBehaviorControlSubscriber, desiredBehaviorSubscriber,
            communicationBridge, yoVariableServer, registry, yoGraphicsListRegistry);

      return ret;
   }

   @DeployableTestMethod(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testDispatchPelvisPoseBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      PelvisPoseBehavior pelvisPoseBehavior = new PelvisPoseBehavior(communicationBridge, yoTime);
      behaviorDispatcher.addHumanoidBehavior(HumanoidBehaviorType.TEST, pelvisPoseBehavior);

      behaviorDispatcher.start();

      HumanoidBehaviorTypePacket requestPelvisPoseBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.TEST);
      behaviorCommunicatorClient.send(requestPelvisPoseBehaviorPacket);
      PrintTools.debug(this, "Requesting PelvisPoseBehavior");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      PelvisPosePacket pelvisPosePacket = createRotationOnlyPelvisPosePacket(new Vector3d(0.0, 1.0, 0.0), Math.toRadians(5.0));
      FramePose desiredPelvisPose = new FramePose();
      desiredPelvisPose.setOrientation(pelvisPosePacket.orientation);

      pelvisPoseBehavior.initialize();
      pelvisPoseBehavior.setInput(pelvisPosePacket);
      assertTrue(pelvisPoseBehavior.hasInputBeenSet());
      PrintTools.debug(this, "Setting PelvisPoseBehavior Input");

      PrintTools.debug(this, "Starting to Excecute Behavior");
      while (!pelvisPoseBehavior.isDone() && yoTime.getDoubleValue() < 20.0)
      {
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
         assertTrue(success);
      }

      assertTrue(pelvisPoseBehavior.isDone());
      assertOrientationsAreWithinThresholds(desiredPelvisPose, getCurrentPelvisPose());

      BambooTools.reportTestFinishedMessage();
   }

   @DeployableTestMethod
   @Test(timeout = 300000)
   public void testDispatchWalkToLocationBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(communicationBridge, fullRobotModel, referenceFrames,
            walkingControllerParameters);
      behaviorDispatcher.addHumanoidBehavior(HumanoidBehaviorType.WALK_TO_OBJECT, walkToLocationBehavior);

      behaviorDispatcher.start();


      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_OBJECT);
      behaviorCommunicatorClient.send(requestWalkToObjectBehaviorPacket);
      PrintTools.debug(this, "Requesting WalkToLocationBehavior");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = 2.0;
      FramePose2d targetMidFeetPose = offsetCurrentRobotMidFeetZUpPose(walkDistance);
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose);
      assertTrue(walkToLocationBehavior.hasInputBeenSet());
      PrintTools.debug(this, "Setting WalkToLocationBehavior Target");

      if (DEBUG)
      {
         ArrayList<Footstep> footsteps = walkToLocationBehavior.getFootSteps();
         Point3d footstepPositionInWorld = new Point3d();
         for (Footstep footStep : footsteps)
         {
            footStep.getPositionInWorldFrame(footstepPositionInWorld);
            PrintTools.debug(this, "" + footstepPositionInWorld);
         }
      }

      PrintTools.debug(this, "Starting to Excecute Behavior");
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

   @DeployableTestMethod
   @Test(timeout = 300000)
   public void testDispatchWalkToLocationBehaviorAndStop() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(communicationBridge, fullRobotModel, referenceFrames,
            walkingControllerParameters);
      behaviorDispatcher.addHumanoidBehavior(HumanoidBehaviorType.WALK_TO_OBJECT, walkToLocationBehavior);

      behaviorDispatcher.start();


      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_OBJECT);
      behaviorCommunicatorClient.send(requestWalkToObjectBehaviorPacket);
      PrintTools.debug(this, "Requesting WalkToLocationBehavior");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = 2.0;
      FramePose2d targetMidFeetPose = offsetCurrentRobotMidFeetZUpPose(walkDistance);
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose);
      assertTrue(walkToLocationBehavior.hasInputBeenSet());
      PrintTools.debug(this, "Setting WalkToLocationBehavior Target");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);
      FramePose2d poseBeforeStop = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);

      HumanoidBehaviorControlModePacket stopModePacket = new HumanoidBehaviorControlModePacket(HumanoidBehaviorControlModeEnum.STOP);
      behaviorCommunicatorClient.send(stopModePacket);
      PrintTools.debug(this, "Sending Stop Request");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);
      FramePose2d poseTwoSecondsAfterStop = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);

      double distanceWalkedAfterStopRequest = poseTwoSecondsAfterStop.getPositionDistance(poseBeforeStop);
      assertTrue(distanceWalkedAfterStopRequest < walkingControllerParameters.getMaxStepLength());
      assertTrue(!walkToLocationBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @DeployableTestMethod
   @Test(timeout = 300000)
   public void testDispatchWalkToLocationBehaviorPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(communicationBridge, fullRobotModel, referenceFrames,
            walkingControllerParameters);
      behaviorDispatcher.addHumanoidBehavior(HumanoidBehaviorType.WALK_TO_OBJECT, walkToLocationBehavior);

      behaviorDispatcher.start();


      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_OBJECT);
      behaviorCommunicatorClient.send(requestWalkToObjectBehaviorPacket);
      PrintTools.debug(this, "Requesting WalkToLocationBehavior");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = 2.0;
      FramePose2d targetMidFeetPose = offsetCurrentRobotMidFeetZUpPose(walkDistance);
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose);
      assertTrue(walkToLocationBehavior.hasInputBeenSet());
      PrintTools.debug(this, "Setting WalkToLocationBehavior Target");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);
      FramePose2d poseBeforePause = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);

      HumanoidBehaviorControlModePacket pauseModePacket = new HumanoidBehaviorControlModePacket(HumanoidBehaviorControlModeEnum.PAUSE);
      behaviorCommunicatorClient.send(pauseModePacket);
      PrintTools.debug(this, "Sending Pause Request");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);
      FramePose2d poseTwoSecondsAfterPause = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);

      double distanceWalkedAfterStopRequest = poseTwoSecondsAfterPause.getPositionDistance(poseBeforePause);
      assertTrue(distanceWalkedAfterStopRequest < walkingControllerParameters.getMaxStepLength());
      assertTrue(!walkToLocationBehavior.isDone());

      HumanoidBehaviorControlModePacket resumeModePacket = new HumanoidBehaviorControlModePacket(HumanoidBehaviorControlModeEnum.RESUME);
      behaviorCommunicatorClient.send(resumeModePacket);
      PrintTools.debug(this, "Sending Resume Request");

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

   private FramePose2d getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(SDFHumanoidRobot robot)
   {
      FramePose midFeetPose = getRobotMidFeetPose(robot);

      FramePose2d ret = new FramePose2d();
      ret.setPoseIncludingFrame(ReferenceFrame.getWorldFrame(), midFeetPose.getX(), midFeetPose.getY(), midFeetPose.getYaw());

      return ret;
   }

   private FramePose getRobotMidFeetPose(SDFHumanoidRobot robot)
   {
      FramePose leftFootPose = getRobotFootPose(robot, RobotSide.LEFT);
      FramePose rightFootPose = getRobotFootPose(robot, RobotSide.RIGHT);

      FramePose ret = new FramePose();
      ret.interpolate(leftFootPose, rightFootPose, 0.5);

      return ret;
   }

   private FramePose getRobotFootPose(SDFHumanoidRobot robot, RobotSide robotSide)
   {
      List<GroundContactPoint> gcPoints = robot.getFootGroundContactPoints(robotSide);
      Joint ankleJoint = gcPoints.get(0).getParentJoint();
      RigidBodyTransform ankleTransformToWorld = new RigidBodyTransform();
      ankleJoint.getTransformToWorld(ankleTransformToWorld);

      FramePose ret = new FramePose();
      ret.setPose(ankleTransformToWorld);

      return ret;
   }

   private PelvisPosePacket createRotationOnlyPelvisPosePacket(Vector3d rotationAxis, double rotationAngle)
   {
      AxisAngle4d desiredPelvisAxisAngle = new AxisAngle4d(rotationAxis, rotationAngle);
      Quat4d desiredPelvisQuat = new Quat4d();
      desiredPelvisQuat.set(desiredPelvisAxisAngle);

      PelvisPosePacket pelvisPosePacket = new PelvisPosePacket(desiredPelvisQuat);
      return pelvisPosePacket;
   }

   private FramePose getCurrentPelvisPose()
   {
      fullRobotModel.updateFrames();
      FramePose ret = new FramePose();
      ret.setToZero(fullRobotModel.getPelvis().getBodyFixedFrame());
      ret.changeFrame(ReferenceFrame.getWorldFrame());

      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose2d framePose1, FramePose2d framePose2)
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

   private void assertOrientationsAreWithinThresholds(FramePose framePose1, FramePose framePose2)
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
