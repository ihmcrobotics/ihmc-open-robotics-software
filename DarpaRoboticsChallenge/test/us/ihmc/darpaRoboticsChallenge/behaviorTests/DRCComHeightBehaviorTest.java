package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ComHeightBehavior;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCComHeightBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

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
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      GlobalTimer.clearTimers();
      TimerTaskScheduler.cancelAndReset();
      AsyncContinuousExecutor.cancelAndReset();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCComHeightBehaviorTest.class + " after class.");
   }

   private static final boolean DEBUG = false;

   public static final double POSITION_THRESHOLD = 0.05;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private double nominalComHeightAboveGround;

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();

      KryoPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10, "DRCControllerCommunicator");
      KryoPacketCommunicator networkObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10, "DRCJunkyCommunicator");

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, networkObjectCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, false, getRobotModel(), controllerCommunicator);
   }

	@AverageDuration(duration = 16.1)
   @Test(timeout = 48271)
   public void testMoveToMinHeight() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = RandomTools.generateRandomDouble(new Random(), 1.0, 3.0);
      double desiredHeightOffset = ComHeightPacket.MIN_COM_HEIGHT;

      testComHeightBehavior(desiredHeightOffset, trajectoryTime);

      BambooTools.reportTestFinishedMessage();
   }

	@AverageDuration(duration = 21.9)
   @Test(timeout = 65777)
   public void testMoveToMaxHeight() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = RandomTools.generateRandomDouble(new Random(), 1.0, 3.0);
      double desiredHeightOffset = ComHeightPacket.MAX_COM_HEIGHT;

      testComHeightBehavior(desiredHeightOffset, trajectoryTime);

      BambooTools.reportTestFinishedMessage();
   }

	@AverageDuration(duration = 16.4)
   @Test(timeout = 49241)
   public void testRandomComHeight() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = RandomTools.generateRandomDouble(new Random(), 1.0, 3.0);
      double desiredHeightOffset = createValidComHeightOffset(RandomTools.generateRandomDouble(new Random(), 0.0, 1.0));

      testComHeightBehavior(desiredHeightOffset, trajectoryTime);

      BambooTools.reportTestFinishedMessage();
   }

   private double createValidComHeightOffset(double relativeOffsetBetweenZeroAndOne)
   {
      double alpha = MathTools.clipToMinMax(relativeOffsetBetweenZeroAndOne, 0.0, 1.0);
      double ret = (ComHeightPacket.MIN_COM_HEIGHT + (ComHeightPacket.MAX_COM_HEIGHT - ComHeightPacket.MIN_COM_HEIGHT) * alpha);

      return ret;
   }

   private void testComHeightBehavior(double desiredHeightOffset, double trajectoryTime) throws SimulationExceededMaximumTimeException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Point3d nominalComPosition = new Point3d();
      drcBehaviorTestHelper.getRobot().computeCenterOfMass(nominalComPosition);
      nominalComHeightAboveGround = nominalComPosition.getZ();

      final ComHeightBehavior comHeightBehavior = new ComHeightBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getYoTime());

      Point3d initialComPoint = new Point3d();
      drcBehaviorTestHelper.getRobot().computeCenterOfMass(initialComPoint);

      ComHeightPacket comHeightPacket = new ComHeightPacket(desiredHeightOffset, trajectoryTime);
      comHeightBehavior.setInput(comHeightPacket);
      assertTrue(comHeightBehavior.hasInputBeenSet());

      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(comHeightBehavior, trajectoryTime + 1.0);
      assertTrue(success);

      Point3d finalComPoint = new Point3d();
      drcBehaviorTestHelper.getRobot().computeCenterOfMass(finalComPoint);

      assertProperComHeightOffsetFromGround(desiredHeightOffset, finalComPoint);
      assertTrue(comHeightBehavior.isDone());
   }

   private void assertProperComHeightOffsetFromGround(double desiredHeightOffset, Point3d finalComPoint)
   {
      double actualHeightOffset = finalComPoint.getZ() - nominalComHeightAboveGround;

      if (DEBUG)
      {
         SysoutTool.println("desiredHeightOffset: " + desiredHeightOffset);
         SysoutTool.println("actualHeightOffset: " + actualHeightOffset);
      }

      assertEquals(desiredHeightOffset, actualHeightOffset, POSITION_THRESHOLD);
   }
}
