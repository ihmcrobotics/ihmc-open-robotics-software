package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.walking.HeadOrientationPacket;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HeadOrientationBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class DRCHeadOrientationBehaviorTest implements MultiRobotTestInterface
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private static final boolean DEBUG = false;

   private final double MAX_ANGLE_TO_TEST_RAD = 15.0 * Math.PI / 180.0;
   private final double POSITION_THRESHOLD = Double.NaN;
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

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      drcSimulationTestHelper = new DRCSimulationTestHelper(testEnvironment, controllerCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, false, getRobotModel());

      Robot robotToTest = drcSimulationTestHelper.getRobot();
      yoTime = robotToTest.getYoTime();

      robot = drcSimulationTestHelper.getRobot();
      fullRobotModel = getRobotModel().createFullRobotModel();

      forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));

      robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder, true);
      controllerCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      PacketCommunicator junkyObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
            "DRCComHeightBehaviorTestJunkyCommunicator");

      communicationBridge = new BehaviorCommunicationBridge(junkyObjectCommunicator, controllerCommunicator, robotToTest.getRobotsYoVariableRegistry());
   }

   //TODO: Fix HeadOrienationManager() so that head actually tracks desired yaw and roll orientations.  Currently, only pitch orientation tracks properly.

   @AverageDuration
   @Test(timeout = 300000)
   public void testHeadPitch() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Vector3d axis = new Vector3d(0, 1, 0);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);

      HeadOrientationPacket headOrientationPacket = createHeadOrientationPacket(axis, rotationAngle);

      HeadOrientationBehavior headOrientationBehavior = testHeadOrientationBehavior(headOrientationPacket, trajectoryTime);

      assertTrue(headOrientationBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   //   @Test(timeout = 300000)
   public void testHeadRoll() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Vector3d axis = new Vector3d(1, 0, 0);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);

      HeadOrientationPacket headOrientationPacket = createHeadOrientationPacket(axis, rotationAngle);

      HeadOrientationBehavior headOrientationBehavior = testHeadOrientationBehavior(headOrientationPacket, trajectoryTime);

      assertTrue(headOrientationBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   //   @Test(timeout = 300000)
   public void testHeadYaw() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Vector3d axis = new Vector3d(0, 0, 1);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);

      HeadOrientationPacket headOrientationPacket = createHeadOrientationPacket(axis, rotationAngle);

      HeadOrientationBehavior headOrientationBehavior = testHeadOrientationBehavior(headOrientationPacket, trajectoryTime);

      assertTrue(headOrientationBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   //   @Test(timeout = 300000)
   public void testRandomOrientation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Quat4d desiredHeadQuat = new Quat4d(RandomTools.generateRandomQuaternion(new Random(), MAX_ANGLE_TO_TEST_RAD));
      HeadOrientationPacket headOrientationPacket = new HeadOrientationPacket(desiredHeadQuat);

      HeadOrientationBehavior headOrientationBehavior = testHeadOrientationBehavior(headOrientationPacket, trajectoryTime);

      assertTrue(headOrientationBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   private HeadOrientationPacket createHeadOrientationPacket(Vector3d axis, double rotationAngle)
   {
      AxisAngle4d desiredAxisAngle = new AxisAngle4d();
      desiredAxisAngle.set(axis, rotationAngle);
      Quat4d desiredHeadQuat = new Quat4d();
      desiredHeadQuat.set(desiredAxisAngle);

      HeadOrientationPacket headOrientationPacket = new HeadOrientationPacket(desiredHeadQuat);
      return headOrientationPacket;
   }

   private HeadOrientationBehavior testHeadOrientationBehavior(HeadOrientationPacket headOrientationPacket, double trajectoryTime)
         throws SimulationExceededMaximumTimeException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final HeadOrientationBehavior headOrientBehavior = new HeadOrientationBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(headOrientBehavior.getControllerGlobalPacketConsumer());

      headOrientBehavior.initialize();
      headOrientBehavior.setInput(headOrientationPacket);
      assertTrue( headOrientBehavior.hasInputBeenSet() );

      FramePose initialHeadPose = getCurrentHeadPose(fullRobotModel);
      success = success && executeBehavior(headOrientBehavior, trajectoryTime);
      FramePose finalHeadPose = getCurrentHeadPose(fullRobotModel);

      if (DEBUG)
      {
         SysoutTool.println(" initial Head Pose :\n" + initialHeadPose);
      }
      FramePose desiredHeadPose = new FramePose();
      desiredHeadPose.setPose(initialHeadPose.getFramePointCopy().getPoint(), headOrientationPacket.quaternion);

      assertPosesAreWithinThresholds(desiredHeadPose, finalHeadPose);

      assertTrue(success);

      return headOrientBehavior;
   }

   private FramePose getCurrentHeadPose(FullRobotModel fullRobotModel)
   {
      FramePose ret = new FramePose();

      fullRobotModel.updateFrames();
      ReferenceFrame headFrame = fullRobotModel.getHead().getBodyFixedFrame();

      ret.setToZero(headFrame);
      ret.changeFrame(ReferenceFrame.getWorldFrame());

      return ret;
   }

   private boolean executeBehavior(final BehaviorInterface behavior, double trajectoryTime) throws SimulationExceededMaximumTimeException
   {
      final double simulationRunTime = trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING;

      if (DEBUG)
      {
         System.out.println("\n");
         SysoutTool.println("starting behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue());
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

                  behavior.doControl();
               }
            }
         }
      };

      behaviorThread.start();

      boolean ret = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);

      if (DEBUG)
      {
         SysoutTool.println("done with behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue());
      }

      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose framePose1, FramePose framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         SysoutTool.println(" desired Head Pose : \n" + framePose1);
         SysoutTool.println(" final Head Pose : \n" + framePose2);

         SysoutTool.println(" positionDistance = " + positionDistance);
         SysoutTool.println(" orientationDistance = " + orientationDistance);
      }

      if (!Double.isNaN(POSITION_THRESHOLD))
      {
         assertEquals(0.0, positionDistance, POSITION_THRESHOLD);
      }
      assertEquals(0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }
}
