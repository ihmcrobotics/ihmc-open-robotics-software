package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

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
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class DRCComHeightBehaviorTest implements MultiRobotTestInterface
{
   private static final boolean DEBUG = false;
   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = false || createMovie;

   private final double POSITION_THRESHOLD = 0.05;
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
   public void testSingleComHeightMove() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final ComHeightBehavior comHeightBehavior = new ComHeightBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(comHeightBehavior.getControllerGlobalPacketConsumer());

      Point3d initialComPoint = new Point3d();
      robot.computeCenterOfMass(initialComPoint);

      double trajectoryTime = RandomTools.generateRandomDouble(new Random(), 1.0, 3.0);
      double desiredHeightOffset = RandomTools.generateRandomDouble(new Random(), 0.8 * ComHeightPacket.MIN_COM_HEIGHT, 0.8 * ComHeightPacket.MAX_COM_HEIGHT);
      ComHeightPacket randomComHeightPacket = new ComHeightPacket(desiredHeightOffset, trajectoryTime); 
      
      comHeightBehavior.initialize();
      comHeightBehavior.setInput(randomComHeightPacket);

      success = success && executeBehavior(comHeightBehavior, trajectoryTime);
      assertTrue(success);

      Point3d finalComPoint = new Point3d();
      robot.computeCenterOfMass(finalComPoint);

      assertProperComHeightChange(comHeightBehavior, initialComPoint, desiredHeightOffset, finalComPoint);

      assertTrue(comHeightBehavior.isDone());
      
      BambooTools.reportTestFinishedMessage();
   }

   //   @Test(timeout = 300000)
   public void testMultipleComHeightMoves() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // First CoM Move
      final ComHeightBehavior comHeightBehavior = new ComHeightBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(comHeightBehavior.getControllerGlobalPacketConsumer());

      Point3d initialComPoint = new Point3d();
      robot.computeCenterOfMass(initialComPoint);

      ComHeightPacket randomComHeightPacket = new ComHeightPacket(new Random());
      comHeightBehavior.initialize();
      comHeightBehavior.setInput(randomComHeightPacket);

      double trajectoryTime = randomComHeightPacket.getTrajectoryTime();
      double desiredHeightOffset = randomComHeightPacket.getHeightOffset();

      assertTrue(comHeightBehavior.isDone());
      
      success = success && executeBehavior(comHeightBehavior, trajectoryTime);

      Point3d finalComPoint = new Point3d();
      robot.computeCenterOfMass(finalComPoint);

      assertProperComHeightChange(comHeightBehavior, initialComPoint, desiredHeightOffset, finalComPoint);
      assertTrue(success);

      // Second CoM Move
      communicationBridge.detachGlobalListenerFromController(comHeightBehavior.getControllerGlobalPacketConsumer());

      final ComHeightBehavior comHeightBehavior2 = new ComHeightBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(comHeightBehavior2.getControllerGlobalPacketConsumer());

      robot.computeCenterOfMass(initialComPoint);

      ComHeightPacket randomComHeightPacket2 = new ComHeightPacket(new Random());
      comHeightBehavior.setInput(randomComHeightPacket2);

      trajectoryTime = randomComHeightPacket2.getTrajectoryTime();
      desiredHeightOffset = randomComHeightPacket2.getHeightOffset();

      assertTrue(comHeightBehavior.isDone());
      
      success = success && executeBehavior(comHeightBehavior2, trajectoryTime);

      robot.computeCenterOfMass(finalComPoint);

      assertProperComHeightChange(comHeightBehavior2, initialComPoint, desiredHeightOffset, finalComPoint);
      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   private void assertProperComHeightChange(final BehaviorInterface comHeightBehavior, Point3d initialComPoint, double desiredHeightOffset,
         Point3d finalComPoint)
   {
      Vector3d comTranslation = new Vector3d();
      comTranslation.sub(finalComPoint, initialComPoint);

      double actualHeightOffset = comTranslation.getZ();

      if (DEBUG)
      {
         SysoutTool.println("desiredHeightOffset: " + desiredHeightOffset);
         SysoutTool.println("actualHeightOffset: " + actualHeightOffset);
      }

      assertEquals(desiredHeightOffset, actualHeightOffset, POSITION_THRESHOLD);

      assertTrue(comHeightBehavior.isDone());
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
