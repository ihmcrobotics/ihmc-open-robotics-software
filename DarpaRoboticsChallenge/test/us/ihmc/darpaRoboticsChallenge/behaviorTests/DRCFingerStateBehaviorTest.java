package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class DRCFingerStateBehaviorTest implements MultiRobotTestInterface
{
   private static final boolean DEBUG = false;
   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = false || createMovie;

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

	@AverageDuration
	@Test(timeout = 300000)
   public void testRandomCloseHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      RobotSide robotSide = RobotSide.LEFT;

      ArrayList<FingerState> closedFingerConfigs = new ArrayList<FingerState>();
      closedFingerConfigs.add(FingerState.CLOSE);
      closedFingerConfigs.add(FingerState.CLOSE_FINGERS);
      closedFingerConfigs.add(FingerState.CLOSE_THUMB);
      closedFingerConfigs.add(FingerState.CRUSH);
      closedFingerConfigs.add(FingerState.CRUSH_INDEX);
      closedFingerConfigs.add(FingerState.CRUSH_MIDDLE);
      closedFingerConfigs.add(FingerState.CRUSH_THUMB);

      FingerState fingerState = closedFingerConfigs.get(RandomTools.generateRandomInt(new Random(), 0, closedFingerConfigs.size() - 1));
      if (DEBUG)
      {
         SysoutTool.println(fingerState.name());
      }
      double trajectoryTime = 3.0;

      SDFJointNameMap jointNameMap = (SDFJointNameMap) fullRobotModel.getRobotSpecificJointNames();
      Joint wristJoint = robot.getJoint(jointNameMap.getJointBeforeHandName(robotSide));

      ArrayList<OneDegreeOfFreedomJoint> fingerJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      wristJoint.recursiveGetOneDegreeOfFreedomJoints(fingerJoints);
      double initialTotalFingerJointQ = 0.0;

      for (OneDegreeOfFreedomJoint fingerJoint : fingerJoints)
      {
         double q = fingerJoint.getQ().getDoubleValue();
         initialTotalFingerJointQ += q;
         if (DEBUG)
         {
            SysoutTool.println(fingerJoint.getName() + " q : " + q);
         }
      }

      FingerStatePacket fingerStatePacket = new FingerStatePacket(robotSide, fingerState);
      FingerStateBehavior fingerStateBehavior = testFingerStateBehavior(fingerStatePacket, trajectoryTime);

      double finalTotalFingerJointQ = 0.0;
      for (OneDegreeOfFreedomJoint fingerJoint : fingerJoints)
      {
         double q = fingerJoint.getQ().getDoubleValue();
         finalTotalFingerJointQ += q;
         if (DEBUG)
         {
            SysoutTool.println(fingerJoint.getName() + " q : " + q);
         }
      }

      double totalFingerJointDeflection = finalTotalFingerJointQ - initialTotalFingerJointQ;

      assertTrue(totalFingerJointDeflection > 0.0);

      assertTrue(fingerStateBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   private FingerStateBehavior testFingerStateBehavior(FingerStatePacket fingerStatePacket, double trajectoryTime)
         throws SimulationExceededMaximumTimeException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final FingerStateBehavior fingerStateBehavior = new FingerStateBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(fingerStateBehavior.getControllerGlobalPacketConsumer());

      fingerStateBehavior.initialize();
      fingerStateBehavior.setInput(fingerStatePacket);

      success = success && executeBehavior(fingerStateBehavior, trajectoryTime);

      assertTrue(success);

      return fingerStateBehavior;
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
