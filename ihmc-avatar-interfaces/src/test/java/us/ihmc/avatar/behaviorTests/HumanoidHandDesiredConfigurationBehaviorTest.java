package us.ihmc.avatar.behaviorTests;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.List;
import java.util.stream.Collectors;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.avatar.testTools.scs2.SCS2BehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandDesiredConfigurationBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

@Disabled
@Deprecated
public abstract class HumanoidHandDesiredConfigurationBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (behaviorTestHelper != null)
      {
         behaviorTestHelper.finishTest();
         behaviorTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterAll
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(HumanoidHandDesiredConfigurationBehaviorTest.class + " after class.");
   }

   private final boolean DEBUG = false;
   private SCS2BehaviorTestHelper behaviorTestHelper;

   @BeforeEach
   public void setUp()
   {
      DefaultCommonAvatarEnvironment testEnvironment = new DefaultCommonAvatarEnvironment();

      SCS2AvatarTestingSimulation simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                                                        testEnvironment,
                                                                                                                        simulationTestingParameters);
      simulationTestHelper.start();
      behaviorTestHelper = new SCS2BehaviorTestHelper(simulationTestHelper);
   }

   @Test
   public void testCloseHand()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);
      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 2.0;

      double fingerJointQInitial = getTotalFingerJointQ(robotSide);
      HandDesiredConfigurationBehavior behavior = testHandDesiredConfigurationBehavior(HumanoidMessageTools.createHandDesiredConfigurationMessage(robotSide,
                                                                                                                                                  HandConfiguration.CLOSE),
                                                                                       trajectoryTime);
      success = behaviorTestHelper.executeBehaviorUntilDone(behavior);
      assertTrue(success);
      double fingerJointQFinal = getTotalFingerJointQ(robotSide);

      LogTools.info("fingerJointQInitial: " + fingerJointQInitial);
      LogTools.info("fingerJointQFinal : " + fingerJointQFinal);

      assertTrue(fingerJointQFinal > fingerJointQInitial);
      assertTrue(behavior.isDone());

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testStopCloseHand()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      LogTools.info("Initializing Simulation");
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 0.3; // [0.3] Hand closes quickly!
      double stopTime = trajectoryTime / 2.0;

      LogTools.info("Initializing Behavior");
      HandDesiredConfigurationBehavior behavior = testHandDesiredConfigurationBehavior(HumanoidMessageTools.createHandDesiredConfigurationMessage(robotSide,
                                                                                                                                                  HandConfiguration.CLOSE),
                                                                                       trajectoryTime);

      LogTools.info("Starting Behavior");
      double fingerJointQInitial = getTotalFingerJointQ(robotSide);
      success = behaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(behavior, stopTime);
      assertTrue(success);
      LogTools.info("Stopping Behavior");
      double fingerJointQAtStop = getTotalFingerJointQ(robotSide);
      behavior.abort();
      assertTrue(!behavior.isDone());

      success = behaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(behavior, 1.0);
      assertTrue(success);
      double fingerJointQFinal = getTotalFingerJointQ(robotSide);

      LogTools.info("fingerJointQInitial: " + fingerJointQInitial);
      LogTools.info("fingerJointQAtStop : " + fingerJointQAtStop);
      LogTools.info("fingerJointQFinal : " + fingerJointQFinal);

      assertTrue(Math.abs(fingerJointQFinal - fingerJointQAtStop) < 3.0);
      assertTrue(!behavior.isDone());

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testPauseAndResumeCloseHand()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      LogTools.info("Initializing Simulation");
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 0.3; // [0.3] Hand closes quickly!
      double stopTime = trajectoryTime / 2.0;

      LogTools.info("Initializing Behavior");
      HandDesiredConfigurationBehavior behavior = testHandDesiredConfigurationBehavior(HumanoidMessageTools.createHandDesiredConfigurationMessage(robotSide,
                                                                                                                                                  HandConfiguration.CLOSE),
                                                                                       trajectoryTime);

      LogTools.info("Starting Behavior");
      double fingerJointQInitial = getTotalFingerJointQ(robotSide);
      success = behaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(behavior, stopTime);
      assertTrue(success);
      LogTools.info("Pausing Behavior");
      double fingerJointQAtPause = getTotalFingerJointQ(robotSide);
      behavior.pause();

      success = behaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(behavior, 1.0);
      assertTrue(success);
      LogTools.info("Resuming Behavior");
      double fingerJointQAtResume = getTotalFingerJointQ(robotSide);
      behavior.resume();
      assertTrue(!behavior.isDone());

      success = behaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(behavior, 1.0);
      assertTrue(success);
      LogTools.info("Behavior Should Be Done");
      double fingerJointQFinal = getTotalFingerJointQ(robotSide);
      behavior.resume();

      LogTools.info("fingerJointQInitial: " + fingerJointQInitial);
      LogTools.info("fingerJointQAtPause : " + fingerJointQAtPause);
      LogTools.info("fingerJointQAtResume : " + fingerJointQAtResume);
      LogTools.info("fingerJointQFinal : " + fingerJointQFinal);

      assertTrue(Math.abs(fingerJointQAtResume - fingerJointQAtPause) < 3.0);
      assertTrue(fingerJointQFinal > fingerJointQAtResume);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private double getTotalFingerJointQ(RobotSide robotSide)
   {
      double ret = 0.0;

      HumanoidJointNameMap jointNameMap = (HumanoidJointNameMap) behaviorTestHelper.getSDFFullRobotModel().getRobotSpecificJointNames();
      List<OneDoFJointReadOnly> fingerJoints = SubtreeStreams.fromChildren(OneDoFJointReadOnly.class,
                                                                           behaviorTestHelper.getRobot().getRigidBody(jointNameMap.getHandName(robotSide)))
                                                             .collect(Collectors.toList());

      for (OneDoFJointReadOnly fingerJoint : fingerJoints)
      {
         double q = fingerJoint.getQ() * getFingerClosedJointAngleSign(robotSide);
         ret += q;
         if (DEBUG)
         {
            LogTools.info(fingerJoint.getName() + " q : " + q);
         }
      }

      return ret;
   }

   private HandDesiredConfigurationBehavior testHandDesiredConfigurationBehavior(HandDesiredConfigurationMessage handDesiredConfigurationMessage,
                                                                                 double trajectoryTime)

   {
      final HandDesiredConfigurationBehavior behavior = new HandDesiredConfigurationBehavior(behaviorTestHelper.getRobotName(),
                                                                                             "test",
                                                                                             behaviorTestHelper.getROS2Node(),
                                                                                             behaviorTestHelper.getYoTime());

      behavior.initialize();
      behavior.setInput(handDesiredConfigurationMessage);
      assertTrue(behavior.hasInputBeenSet());

      return behavior;
   }

   protected double getFingerClosedJointAngleSign(RobotSide robotSide)
   {
      return 1.0;
   }
}
