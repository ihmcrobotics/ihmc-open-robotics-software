package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.communication.packets.walking.FootPosePacket;
import us.ihmc.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCFootPoseBehaviorTest implements MultiRobotTestInterface
{
   private final static boolean KEEP_SCS_UP = false;

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
      if (KEEP_SCS_UP)
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
   
   private static final boolean DEBUG = false;
  
   private final double POSITION_THRESHOLD = 0.1;
   private final double ORIENTATION_THRESHOLD = 0.007;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private final DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();
   private final PacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
         "DRCHandPoseBehaviorTestControllerCommunicator");

   private DoubleYoVariable yoTime;

   private RobotDataReceiver robotDataReceiver;
   private ForceSensorDataHolder forceSensorDataHolder;

   private BehaviorCommunicationBridge communicationBridge;

   private SDFRobot robot;
   private FullRobotModel fullRobotModel;
   private ReferenceFrames referenceFrames;

   private CapturePointUpdatable capturePointUpdatable;
   private BooleanYoVariable yoDoubleSupport;

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      drcSimulationTestHelper = new DRCSimulationTestHelper(testEnvironment, controllerCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, false, getRobotModel());

      Robot robotToTest = drcSimulationTestHelper.getRobot();
      yoTime = robotToTest.getYoTime();

      robot = drcSimulationTestHelper.getRobot();
      fullRobotModel = getRobotModel().createFullRobotModel();

      forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));

      robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder, true);
      controllerCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);
      referenceFrames = robotDataReceiver.getReferenceFrames();

      PacketCommunicator junkyObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
            "DRCComHeightBehaviorTestJunkyCommunicator");

      communicationBridge = new BehaviorCommunicationBridge(junkyObjectCommunicator, controllerCommunicator, robotToTest.getRobotsYoVariableRegistry());

      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      controllerCommunicator.attachListener(CapturabilityBasedStatus.class, capturabilityBasedStatusSubsrciber);
      capturePointUpdatable = new CapturePointUpdatable(capturabilityBasedStatusSubsrciber, new YoGraphicsListRegistry(), robot.getRobotsYoVariableRegistry());
      yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();
   }

	@AverageDuration
	@Test(timeout = 300000)
   public void testSimpleFootPoseBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double trajectoryTime = 3.0;
      RobotSide robotSide = RobotSide.LEFT;
      double deltaZ = 0.2;

      final FootPoseBehavior footPoseBehavior = new FootPoseBehavior(communicationBridge, yoTime, yoDoubleSupport);
      communicationBridge.attachGlobalListenerToController(footPoseBehavior.getControllerGlobalPacketConsumer());

      FramePose desiredFootPose = getCurrentFootPose(robotSide);
      desiredFootPose.setZ(desiredFootPose.getZ() + deltaZ);

      FootPosePacket desiredFootPosePacket = createFootPosePacket(robotSide, desiredFootPose, trajectoryTime);
      footPoseBehavior.initialize();
      footPoseBehavior.setInput(desiredFootPosePacket);

      ArrayList<BehaviorInterface> behaviors = new ArrayList<BehaviorInterface>();
      behaviors.add(footPoseBehavior);

      success = executeBehaviors(behaviors, trajectoryTime);
      assertTrue(success);

      FramePose finalFootPose = getCurrentFootPose(robotSide);
      assertTrue(footPoseBehavior.isDone());
      assertPosesAreWithinThresholds(desiredFootPose, finalFootPose);

      BambooTools.reportTestFinishedMessage();
   }

	@AverageDuration
	@Test(timeout = 300000)
   public void testSimulataneousLeftAndRightFootPoses() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double trajectoryTime = 3.0;
      double deltaZ = 0.2;

      SideDependentList<FramePose> initialFootPoses = new SideDependentList<FramePose>();
      SideDependentList<FootPoseBehavior> footPoseBehaviors = new SideDependentList<FootPoseBehavior>();
      LinkedHashMap<FootPoseBehavior, FramePose> desiredFootPoses = new LinkedHashMap<FootPoseBehavior, FramePose>();

      RobotSide lastRobotSideSentToController = null;

      for (RobotSide robotSide : RobotSide.values)
      {
         final FootPoseBehavior footPoseBehavior = new FootPoseBehavior(communicationBridge, yoTime, yoDoubleSupport);
         communicationBridge.attachGlobalListenerToController(footPoseBehavior.getControllerGlobalPacketConsumer());

         FramePose initialFootPose = getCurrentFootPose(robotSide);
         initialFootPoses.put(robotSide, initialFootPose);

         FramePose desiredFootPose = new FramePose(initialFootPose);
         desiredFootPose.setZ(desiredFootPose.getZ() + deltaZ);

         FootPosePacket desiredFootPosePacket = createFootPosePacket(robotSide, desiredFootPose, trajectoryTime);
         footPoseBehavior.initialize();
         footPoseBehavior.setInput(desiredFootPosePacket);

         footPoseBehaviors.put(robotSide, footPoseBehavior);
         desiredFootPoses.put(footPoseBehavior, desiredFootPose);

         lastRobotSideSentToController = robotSide;
      }

      success = executeBehaviors(footPoseBehaviors, trajectoryTime);
      assertTrue(success);

      assertFootPoseCompletedSuccessfully(lastRobotSideSentToController, footPoseBehaviors, desiredFootPoses);
      assertFootPoseDidNotChange(lastRobotSideSentToController.getOppositeSide(), footPoseBehaviors, initialFootPoses);

      BambooTools.reportTestFinishedMessage();
   }

   private void assertFootPoseCompletedSuccessfully(RobotSide robotSide, SideDependentList<FootPoseBehavior> footPoseBehaviors,
         LinkedHashMap<FootPoseBehavior, FramePose> desiredFootPoses)
   {
      FramePose finalFootPose = getCurrentFootPose(robotSide);

      FootPoseBehavior footPoseBehavior = footPoseBehaviors.get(robotSide);
      FramePose desiredFootPose = desiredFootPoses.get(footPoseBehavior);

      assertTrue(footPoseBehavior.isDone());
      assertPosesAreWithinThresholds(desiredFootPose, finalFootPose);
   }

   private void assertFootPoseDidNotChange(RobotSide robotSide, SideDependentList<FootPoseBehavior> footPoseBehaviors, SideDependentList<FramePose> initialFootPoses)
   {
      FramePose finalFootPose = getCurrentFootPose(robotSide);

      FootPoseBehavior footPoseBehavior = footPoseBehaviors.get(robotSide);
      FramePose initialFootPose = initialFootPoses.get(robotSide);

      assertTrue(footPoseBehavior.isDone());
      assertPosesAreWithinThresholds(finalFootPose, initialFootPose);
   }

   private FootPosePacket createFootPosePacket(RobotSide robotSide, FramePose desiredFootPose, double trajectoryTime)
   {
      Point3d desiredFootPosition = new Point3d();
      Quat4d desiredFootOrientation = new Quat4d();

      desiredFootPose.getPosition(desiredFootPosition);
      desiredFootPose.getOrientation(desiredFootOrientation);

      FootPosePacket ret = new FootPosePacket(robotSide, desiredFootPosition, desiredFootOrientation, trajectoryTime);

      return ret;
   }

   private FramePose getCurrentFootPose(RobotSide robotSide)
   {
      robotDataReceiver.updateRobotModel();
      ReferenceFrame footFrame = fullRobotModel.getFoot(robotSide).getBodyFixedFrame();
      FramePose footPose = new FramePose();
      footPose.setToZero(footFrame);
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      return footPose;
   }

   private boolean executeBehaviors(final SideDependentList<FootPoseBehavior> behaviorsSideDependentList, final double simulationRunTime)
         throws SimulationExceededMaximumTimeException
   {
      ArrayList<BehaviorInterface> behaviors = new ArrayList<BehaviorInterface>();

      for (RobotSide robotSide : RobotSide.values)
      {
         BehaviorInterface handPoseBehavior = behaviorsSideDependentList.get(robotSide);
         behaviors.add(handPoseBehavior);
      }

      boolean ret = executeBehaviors(behaviors, simulationRunTime);

      return ret;
   }

   private boolean executeBehaviors(final ArrayList<BehaviorInterface> behaviors, double trajectoryTime) throws SimulationExceededMaximumTimeException
   {
      final double simulationRunTime = trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING;

      if (DEBUG)
      {
         System.out.println("\n");
         for (BehaviorInterface behavior : behaviors)
         {
            SysoutTool.println("starting behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue());
         }
      }

      Thread behaviorThread = new Thread()
      {
         public void run()
         {
            {
               double startTime = Double.NaN;
               boolean simStillRunning = true;
               boolean initalized = false;

               while (simStillRunning)
               {
                  if (!initalized)
                  {
                     startTime = yoTime.getDoubleValue();
                     initalized = true;
                  }

                  double timeSpentSimulating = yoTime.getDoubleValue() - startTime;
                  simStillRunning = timeSpentSimulating < simulationRunTime;

                  for (BehaviorInterface behavior : behaviors)
                  {
                     behavior.doControl();
                  }
                  capturePointUpdatable.update(yoTime.getDoubleValue());
               }
            }
         }
      };

      behaviorThread.start();

      boolean ret = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);

      if (DEBUG)
      {
         for (BehaviorInterface behavior : behaviors)
         {
            SysoutTool.println("done with behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue());
         }
      }

      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose framePose1, FramePose framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         System.out.println("testSimpleHandPoseMove: positionDistance=" + positionDistance);
         System.out.println("testSimpleHandPoseMove: orientationDistance=" + orientationDistance);
      }

      assertEquals(0.0, positionDistance, POSITION_THRESHOLD);
      assertEquals(0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }
}
