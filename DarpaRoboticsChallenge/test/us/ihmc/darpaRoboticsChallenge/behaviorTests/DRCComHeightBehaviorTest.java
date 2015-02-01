package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.Random;

import javax.vecmath.Point3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ComHeightBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.Robot;
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
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCComHeightBehaviorTest implements MultiRobotTestInterface
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
   

   public static final double POSITION_THRESHOLD = 0.05;
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
   private double nominalComHeightAboveGround;

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

   @AverageDuration
   @Test(timeout = 300000)
   public void testRandomComHeight() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      Point3d nominalComPosition = new Point3d();
      robot.computeCenterOfMass(nominalComPosition);
      nominalComHeightAboveGround = nominalComPosition.getZ();
      
      double trajectoryTime = RandomTools.generateRandomDouble(new Random(), 1.0, 3.0);
      double desiredHeightOffset = createValidComHeightOffset(RandomTools.generateRandomDouble(new Random(), 0.0, 1.0));

      final ComHeightBehavior comHeightBehavior = new ComHeightBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(comHeightBehavior.getControllerGlobalPacketConsumer());

      testComHeightBehavior(comHeightBehavior, desiredHeightOffset, trajectoryTime);

      BambooTools.reportTestFinishedMessage();
   }
   
	@AverageDuration
	@Test(timeout = 300000)
   public void testMoveToMinHeight() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      Point3d nominalComPosition = new Point3d();
      robot.computeCenterOfMass(nominalComPosition);
      nominalComHeightAboveGround = nominalComPosition.getZ();
      
      double trajectoryTime = RandomTools.generateRandomDouble(new Random(), 1.0, 3.0);
      double desiredHeightOffset = ComHeightPacket.MIN_COM_HEIGHT;

      final ComHeightBehavior comHeightBehavior = new ComHeightBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(comHeightBehavior.getControllerGlobalPacketConsumer());

      testComHeightBehavior(comHeightBehavior, desiredHeightOffset, trajectoryTime);

      BambooTools.reportTestFinishedMessage();
   }
	
	  @AverageDuration
	   @Test(timeout = 300000)
	   public void testMoveToMaxHeight() throws SimulationExceededMaximumTimeException
	   {
	      BambooTools.reportTestStartedMessage();

	      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
	      assertTrue(success);
	      Point3d nominalComPosition = new Point3d();
	      robot.computeCenterOfMass(nominalComPosition);
	      nominalComHeightAboveGround = nominalComPosition.getZ();
	      
	      double trajectoryTime = RandomTools.generateRandomDouble(new Random(), 1.0, 3.0);
	      double desiredHeightOffset = ComHeightPacket.MAX_COM_HEIGHT;

	      final ComHeightBehavior comHeightBehavior = new ComHeightBehavior(communicationBridge, yoTime);
	      communicationBridge.attachGlobalListenerToController(comHeightBehavior.getControllerGlobalPacketConsumer());

	      testComHeightBehavior(comHeightBehavior, desiredHeightOffset, trajectoryTime);

	      BambooTools.reportTestFinishedMessage();
	   }

	@AverageDuration
	@Test(timeout = 300000)
   public void testTwoComHeightMovesUsingOneBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      Point3d nominalComPosition = new Point3d();
      robot.computeCenterOfMass(nominalComPosition);
      nominalComHeightAboveGround = nominalComPosition.getZ();

      double trajectoryTime = RandomTools.generateRandomDouble(new Random(), 1.0, 3.0);
      double desiredHeightOffset = RandomTools.generateRandomDouble(new Random(), 0.8 * ComHeightPacket.MIN_COM_HEIGHT, 0.8 * ComHeightPacket.MAX_COM_HEIGHT);

      final ComHeightBehavior comHeightBehavior = new ComHeightBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(comHeightBehavior.getControllerGlobalPacketConsumer());

      testComHeightBehavior(comHeightBehavior, desiredHeightOffset, trajectoryTime);

      testComHeightBehavior(comHeightBehavior, desiredHeightOffset, trajectoryTime);

      BambooTools.reportTestFinishedMessage();
   }

	@AverageDuration
	@Test(timeout = 300000)
   public void testTwoComHeightMovesUsingTwoBehaviors() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      Point3d nominalComPosition = new Point3d();
      robot.computeCenterOfMass(nominalComPosition);
      nominalComHeightAboveGround = nominalComPosition.getZ();

      double trajectoryTime = RandomTools.generateRandomDouble(new Random(), 1.0, 3.0);
      double desiredHeightOffset = RandomTools.generateRandomDouble(new Random(), 0.8 * ComHeightPacket.MIN_COM_HEIGHT, 0.8 * ComHeightPacket.MAX_COM_HEIGHT);

      final ComHeightBehavior comHeightBehavior = new ComHeightBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(comHeightBehavior.getControllerGlobalPacketConsumer());

      testComHeightBehavior(comHeightBehavior, desiredHeightOffset, trajectoryTime);

      final ComHeightBehavior comHeightBehavior2 = new ComHeightBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(comHeightBehavior2.getControllerGlobalPacketConsumer());

      testComHeightBehavior(comHeightBehavior2, desiredHeightOffset, trajectoryTime);

      BambooTools.reportTestFinishedMessage();
   }

   private double createValidComHeightOffset(double relativeOffsetBetweenZeroAndOne)
   {
      double alpha = MathTools.clipToMinMax(relativeOffsetBetweenZeroAndOne, 0.0, 1.0);
      double ret = (ComHeightPacket.MIN_COM_HEIGHT + (ComHeightPacket.MAX_COM_HEIGHT - ComHeightPacket.MIN_COM_HEIGHT) * alpha);
      
      return ret;
   }
	
   private void testComHeightBehavior(final ComHeightBehavior comHeightBehavior, double desiredHeightOffset, double trajectoryTime)
         throws SimulationExceededMaximumTimeException
   {
      Point3d initialComPoint = new Point3d();
      robot.computeCenterOfMass(initialComPoint);

      ComHeightPacket comHeightPacket = new ComHeightPacket(desiredHeightOffset, trajectoryTime);
      comHeightBehavior.setInput(comHeightPacket);

      boolean success = executeBehavior(comHeightBehavior, trajectoryTime);
      assertTrue(success);

      Point3d finalComPoint = new Point3d();
      robot.computeCenterOfMass(finalComPoint);

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
}
