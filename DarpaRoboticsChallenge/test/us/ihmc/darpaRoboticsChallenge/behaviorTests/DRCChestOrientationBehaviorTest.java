package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestOrientationBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class DRCChestOrientationBehaviorTest implements MultiRobotTestInterface
{
   private static final boolean DEBUG = false;
   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = false || createMovie;

   private final double MAX_ANGLE_TO_TEST_RAD = 30.0 * Math.PI / 180.0;
   private final double POSITION_THRESHOLD = Double.NaN;
   private final double ORIENTATION_THRESHOLD = 0.007;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private final DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();
   private final PacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
         "DRCHandPoseBehaviorTestControllerCommunicator");

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private DoubleYoVariable yoTime;

   private RobotDataReceiver robotDataReceiver;
   private ForceSensorDataHolder forceSensorDataHolder;

   private BehaviorCommunicationBridge communicationBridge;

   private SDFRobot robot;
   private FullRobotModel fullRobotModel;
   private ReferenceFrames referenceFrames;

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      drcSimulationTestHelper = new DRCSimulationTestHelper(testEnvironment, controllerCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, checkNothingChanged, showGUI, createMovie, false, getRobotModel());

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
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   @Test(timeout = 300000)
   public void testSingleRandomChestOrientationMove() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      Quat4d desiredChestQuat = new Quat4d(RandomTools.generateRandomQuaternion(new Random(), MAX_ANGLE_TO_TEST_RAD));
      ChestOrientationPacket chestOrientationPacket = new ChestOrientationPacket(desiredChestQuat, 1.0);
      testChestOrientationBehavior(chestOrientationPacket);

      BambooTools.reportTestFinishedMessage();
   }

   @Test(timeout = 300000)
   public void testSingleChestPitchOrientationMove() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);
      AxisAngle4d desiredAxisAngle = new AxisAngle4d(0, 1, 0, rotationAngle);
      Quat4d desiredChestQuat = new Quat4d();
      desiredChestQuat.set(desiredAxisAngle);

      ChestOrientationPacket chestOrientationPacket = new ChestOrientationPacket(desiredChestQuat, 1.0);
     
      testChestOrientationBehavior(chestOrientationPacket);
      
      BambooTools.reportTestFinishedMessage();
   }

   @Test(timeout = 300000)
   public void testSingleChestRollOrientationMove() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);
      AxisAngle4d desiredAxisAngle = new AxisAngle4d(1, 0, 0, rotationAngle);
      Quat4d desiredChestQuat = new Quat4d();
      desiredChestQuat.set(desiredAxisAngle);

      ChestOrientationPacket chestOrientationPacket = new ChestOrientationPacket(desiredChestQuat, 1.0);
     
      testChestOrientationBehavior(chestOrientationPacket);
      
      BambooTools.reportTestFinishedMessage();
   }

   @Test(timeout = 300000)
   public void testSingleChestYawOrientationMove() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);
      AxisAngle4d desiredAxisAngle = new AxisAngle4d(0, 0, 1, rotationAngle);
      Quat4d desiredChestQuat = new Quat4d();
      desiredChestQuat.set(desiredAxisAngle);

      ChestOrientationPacket chestOrientationPacket = new ChestOrientationPacket(desiredChestQuat, 1.0);
     
      testChestOrientationBehavior(chestOrientationPacket);
      
      BambooTools.reportTestFinishedMessage();
   }
   
   private void testChestOrientationBehavior(ChestOrientationPacket chestOrientationPacket) throws SimulationExceededMaximumTimeException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final ChestOrientationBehavior chestOrientBehavior = new ChestOrientationBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(chestOrientBehavior.getControllerGlobalPacketConsumer());

      chestOrientBehavior.setInput(chestOrientationPacket);

      FramePose initialChestPose = getCurrentChestPose(fullRobotModel);
      success = success && executeBehavior(chestOrientBehavior, chestOrientationPacket.getTrajectoryTime());
      FramePose finalChestPose = getCurrentChestPose(fullRobotModel);

      if (DEBUG)
      {
         SysoutTool.println(" initial Chest Pose :\n" + initialChestPose);
      }
      FramePose desiredChestPose = new FramePose();
      desiredChestPose.setPose(initialChestPose.getFramePointCopy().getPoint(), chestOrientationPacket.quaternion);
      assertPosesAreWithinThresholds(desiredChestPose, finalChestPose);

      assertTrue(success);
   }

   private FramePose getCurrentChestPose(FullRobotModel fullRobotModel)
   {
      fullRobotModel.updateFrames();
      FramePose ret = new FramePose();
      ret.setToZero(fullRobotModel.getChest().getBodyFixedFrame());
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
         SysoutTool.println(" desired Chest Pose :\n" + framePose1);
         SysoutTool.println(" actual Chest Pose :\n" + framePose2);

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
