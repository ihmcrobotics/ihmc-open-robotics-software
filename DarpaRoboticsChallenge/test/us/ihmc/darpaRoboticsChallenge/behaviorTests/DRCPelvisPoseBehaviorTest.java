package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
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
import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class DRCPelvisPoseBehaviorTest implements MultiRobotTestInterface
{
   private static final boolean DEBUG = false;
   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = false || createMovie;

   private final double MAX_ANGLE_TO_TEST_RAD = 15.0 * Math.PI / 180.0;
   private final double MAX_TRANSLATION_TO_TEST_M = 0.15;

   private final double POSITION_THRESHOLD = 0.05;
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
   public void testSingleRandomPelvisRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Quat4d desiredPelvisQuat = new Quat4d(RandomTools.generateRandomQuaternion(new Random(), 0.8 * MAX_ANGLE_TO_TEST_RAD));
      PelvisPosePacket pelvisPosePacket = new PelvisPosePacket(desiredPelvisQuat);

      PelvisPoseBehavior pelvisPoseBehavior = testPelvisPoseBehavior(pelvisPosePacket);

      assertTrue(pelvisPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @Test(timeout = 300000)
   public void testPelvisPitchRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Vector3d rotationAxis = new Vector3d(0, 1, 0);
      double rotationAngle = RandomTools.generateRandomDouble(new Random(), MAX_ANGLE_TO_TEST_RAD);
      PelvisPosePacket pelvisPosePacket = createRotationOnlyPelvisPosePacket(rotationAxis, rotationAngle);

      PelvisPoseBehavior pelvisPoseBehavior = testPelvisPoseBehavior(pelvisPosePacket);

      assertTrue(pelvisPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

//   @Test(timeout = 300000)
   public void testPelvisRollRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Vector3d rotationAxis = new Vector3d(1, 0, 0);
      double rotationAngle = RandomTools.generateRandomDouble(new Random(), MAX_ANGLE_TO_TEST_RAD);
      PelvisPosePacket pelvisPosePacket = createRotationOnlyPelvisPosePacket(rotationAxis, rotationAngle);

      PelvisPoseBehavior pelvisPoseBehavior = testPelvisPoseBehavior(pelvisPosePacket);

      assertTrue(pelvisPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @Test(timeout = 300000)
   public void testPelvisYawRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Vector3d rotationAxis = new Vector3d(0, 0, 1);
      double rotationAngle = RandomTools.generateRandomDouble(new Random(), 0.8 * MAX_ANGLE_TO_TEST_RAD);
      PelvisPosePacket pelvisPosePacket = createRotationOnlyPelvisPosePacket(rotationAxis, rotationAngle);

      PelvisPoseBehavior pelvisPoseBehavior = testPelvisPoseBehavior(pelvisPosePacket);

      assertTrue(pelvisPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

      @Test(timeout = 300000)
   public void testPelvisXTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Vector3d desiredDirection = new Vector3d(1, 0, 0);
      FramePose currentPelvisPose = getCurrentPelvisPose(fullRobotModel);
      PelvisPosePacket pelvisPosePacket = createTranslationOnlyPelvisPosePacket(desiredDirection, currentPelvisPose);

      PelvisPoseBehavior pelvisPoseBehavior = testPelvisPoseBehavior(pelvisPosePacket);

      assertTrue(pelvisPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

      @Test(timeout = 300000)
   public void testPelvisYTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Vector3d desiredDirection = new Vector3d(0, 1, 0);
      FramePose currentPelvisPose = getCurrentPelvisPose(fullRobotModel);
      PelvisPosePacket pelvisPosePacket = createTranslationOnlyPelvisPosePacket(desiredDirection, currentPelvisPose);

      PelvisPoseBehavior pelvisPoseBehavior = testPelvisPoseBehavior(pelvisPosePacket);

      assertTrue(pelvisPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

//      @Test(timeout = 300000)
   public void testPelvisZTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Vector3d desiredDirection = new Vector3d(0, 0, 1);
      FramePose currentPelvisPose = getCurrentPelvisPose(fullRobotModel);
      PelvisPosePacket pelvisPosePacket = createTranslationOnlyPelvisPosePacket(desiredDirection, currentPelvisPose);

      PelvisPoseBehavior pelvisPoseBehavior = testPelvisPoseBehavior(pelvisPosePacket);

      assertTrue(pelvisPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   private PelvisPosePacket createRotationOnlyPelvisPosePacket(Vector3d rotationAxis, double rotationAngle)
   {
      AxisAngle4d desiredPelvisAxisAngle = new AxisAngle4d(rotationAxis, rotationAngle);
      Quat4d desiredPelvisQuat = new Quat4d();
      desiredPelvisQuat.set(desiredPelvisAxisAngle);

      PelvisPosePacket pelvisPosePacket = new PelvisPosePacket(desiredPelvisQuat);
      return pelvisPosePacket;
   }

   private PelvisPosePacket createTranslationOnlyPelvisPosePacket(Vector3d desiredDirection, FramePose currentPelvisPose)
   {
      desiredDirection.normalize();
      double distanceToTranslate = RandomTools.generateRandomDouble(new Random(), MAX_TRANSLATION_TO_TEST_M);
      desiredDirection.scale(distanceToTranslate);
      Point3d desiredPelvisPoint = new Point3d(currentPelvisPose.getFramePointCopy().getPoint());
      desiredPelvisPoint.add(desiredDirection);

      PelvisPosePacket pelvisPosePacket = new PelvisPosePacket(desiredPelvisPoint);
      return pelvisPosePacket;
   }

   private PelvisPoseBehavior testPelvisPoseBehavior(PelvisPosePacket pelvisPosePacket) throws SimulationExceededMaximumTimeException
   {
      final PelvisPoseBehavior pelvisPoseBehavior = new PelvisPoseBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(pelvisPoseBehavior.getControllerGlobalPacketConsumer());
      
      pelvisPoseBehavior.initialize();
      pelvisPoseBehavior.setInput(pelvisPosePacket);

      FramePose initialPelvisPose = getCurrentPelvisPose(fullRobotModel);
      boolean success = executeBehavior(pelvisPoseBehavior, pelvisPosePacket.getTrajectoryTime());
      FramePose finalPelvisPose = getCurrentPelvisPose(fullRobotModel);

      if (DEBUG)
      {
         SysoutTool.println(" initial Pelvis Pose :\n" + initialPelvisPose + "\n");
      }
      FramePose desiredPelvisPose = new FramePose();

      if (pelvisPosePacket.point == null)
      {
         desiredPelvisPose.setOrientation(pelvisPosePacket.quaternion);
         assertOrientationsAreWithinThresholds(desiredPelvisPose, finalPelvisPose);
      }
      else if (pelvisPosePacket.quaternion == null)
      {
         desiredPelvisPose.setPosition(pelvisPosePacket.point);
         assertPositionsAreWithinThresholds(desiredPelvisPose, finalPelvisPose);
      }
      else
      {
         desiredPelvisPose.setPose(pelvisPosePacket.point, pelvisPosePacket.quaternion);
         assertPosesAreWithinThresholds(desiredPelvisPose, finalPelvisPose);
      }

      assertTrue(success);
      return pelvisPoseBehavior;
   }
   
   private FramePose getCurrentPelvisPose(FullRobotModel fullRobotModel)
   {
      FramePose ret = new FramePose();
      ret.setToZero(fullRobotModel.getPelvis().getBodyFixedFrame());
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
         SysoutTool.println(" desired Pelvis Pose :\n" + framePose1 + "\n");
         SysoutTool.println(" actual Pelvis Pose :\n" + framePose2 + "\n");

         SysoutTool.println(" positionDistance = " + positionDistance);
         SysoutTool.println(" orientationDistance = " + orientationDistance);
      }

      if (!Double.isNaN(POSITION_THRESHOLD))
      {
         assertEquals(0.0, positionDistance, POSITION_THRESHOLD);
      }
      assertEquals(0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }

   private void assertPositionsAreWithinThresholds(FramePose framePose1, FramePose framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);

      if (DEBUG)
      {
         SysoutTool.println(" desired Pelvis Pose :\n" + framePose1 + "\n");
         SysoutTool.println(" actual Pelvis Pose :\n" + framePose2 + "\n");

         SysoutTool.println(" positionDistance = " + positionDistance);
      }

      if (!Double.isNaN(POSITION_THRESHOLD))
      {
         assertEquals(0.0, positionDistance, POSITION_THRESHOLD);
      }
   }

   private void assertOrientationsAreWithinThresholds(FramePose framePose1, FramePose framePose2)
   {
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         SysoutTool.println(" desired Pelvis Pose :\n" + framePose1 + "\n");
         SysoutTool.println(" actual Pelvis Pose :\n" + framePose2 + "\n");

         SysoutTool.println(" orientationDistance = " + orientationDistance);
      }

      assertEquals(0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }
}
