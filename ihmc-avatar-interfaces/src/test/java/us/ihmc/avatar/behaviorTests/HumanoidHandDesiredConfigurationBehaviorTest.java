package us.ihmc.avatar.behaviorTests;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandDesiredConfigurationBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterAll
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(HumanoidHandDesiredConfigurationBehaviorTest.class + " after class.");
   }

   private final boolean DEBUG = false;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @BeforeEach
   public void setUp()
   {
      DefaultCommonAvatarEnvironment testEnvironment = new DefaultCommonAvatarEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), DRCObstacleCourseStartingLocation.DEFAULT,
                                                        simulationTestingParameters, getRobotModel());
   }

   @Test
   public void testCloseHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 2.0;

      double fingerJointQInitial = getTotalFingerJointQ(robotSide);
      HandDesiredConfigurationBehavior behavior = testHandDesiredConfigurationBehavior(HumanoidMessageTools.createHandDesiredConfigurationMessage(robotSide,
                                                                                                                                                  HandConfiguration.CLOSE),
                                                                                       trajectoryTime);
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(behavior);
      assertTrue(success);
      double fingerJointQFinal = getTotalFingerJointQ(robotSide);

      PrintTools.debug(this, "fingerJointQInitial: " + fingerJointQInitial);
      PrintTools.debug(this, "fingerJointQFinal : " + fingerJointQFinal);

      assertTrue(fingerJointQFinal > fingerJointQInitial);
      assertTrue(behavior.isDone());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testStopCloseHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      PrintTools.debug(this, "Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 0.3; // [0.3] Hand closes quickly!
      double stopTime = trajectoryTime / 2.0;

      PrintTools.debug(this, "Initializing Behavior");
      HandDesiredConfigurationBehavior behavior = testHandDesiredConfigurationBehavior(HumanoidMessageTools.createHandDesiredConfigurationMessage(robotSide,
                                                                                                                                                  HandConfiguration.CLOSE),
                                                                                       trajectoryTime);

      PrintTools.debug(this, "Starting Behavior");
      double fingerJointQInitial = getTotalFingerJointQ(robotSide);
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(behavior, stopTime);
      assertTrue(success);
      PrintTools.debug(this, "Stopping Behavior");
      double fingerJointQAtStop = getTotalFingerJointQ(robotSide);
      behavior.abort();
      assertTrue(!behavior.isDone());

      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(behavior, 1.0);
      assertTrue(success);
      double fingerJointQFinal = getTotalFingerJointQ(robotSide);

      PrintTools.debug(this, "fingerJointQInitial: " + fingerJointQInitial);
      PrintTools.debug(this, "fingerJointQAtStop : " + fingerJointQAtStop);
      PrintTools.debug(this, "fingerJointQFinal : " + fingerJointQFinal);

      assertTrue(Math.abs(fingerJointQFinal - fingerJointQAtStop) < 3.0);
      assertTrue(!behavior.isDone());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testPauseAndResumeCloseHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      PrintTools.debug(this, "Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 0.3; // [0.3] Hand closes quickly!
      double stopTime = trajectoryTime / 2.0;

      PrintTools.debug(this, "Initializing Behavior");
      HandDesiredConfigurationBehavior behavior = testHandDesiredConfigurationBehavior(HumanoidMessageTools.createHandDesiredConfigurationMessage(robotSide,
                                                                                                                                                  HandConfiguration.CLOSE),
                                                                                       trajectoryTime);

      PrintTools.debug(this, "Starting Behavior");
      double fingerJointQInitial = getTotalFingerJointQ(robotSide);
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(behavior, stopTime);
      assertTrue(success);
      PrintTools.debug(this, "Pausing Behavior");
      double fingerJointQAtPause = getTotalFingerJointQ(robotSide);
      behavior.pause();

      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(behavior, 1.0);
      assertTrue(success);
      PrintTools.debug(this, "Resuming Behavior");
      double fingerJointQAtResume = getTotalFingerJointQ(robotSide);
      behavior.resume();
      assertTrue(!behavior.isDone());

      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(behavior, 1.0);
      assertTrue(success);
      PrintTools.debug(this, "Behavior Should Be Done");
      double fingerJointQFinal = getTotalFingerJointQ(robotSide);
      behavior.resume();

      PrintTools.debug(this, "fingerJointQInitial: " + fingerJointQInitial);
      PrintTools.debug(this, "fingerJointQAtPause : " + fingerJointQAtPause);
      PrintTools.debug(this, "fingerJointQAtResume : " + fingerJointQAtResume);
      PrintTools.debug(this, "fingerJointQFinal : " + fingerJointQFinal);

      assertTrue(Math.abs(fingerJointQAtResume - fingerJointQAtPause) < 3.0);
      assertTrue(fingerJointQFinal > fingerJointQAtResume);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private double getTotalFingerJointQ(RobotSide robotSide)
   {
      double ret = 0.0;

      ArrayList<OneDegreeOfFreedomJoint> fingerJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      HumanoidJointNameMap jointNameMap = (HumanoidJointNameMap) drcBehaviorTestHelper.getSDFFullRobotModel().getRobotSpecificJointNames();
      Joint wristJoint = drcBehaviorTestHelper.getRobot().getJoint(jointNameMap.getJointBeforeHandName(robotSide));
      wristJoint.recursiveGetOneDegreeOfFreedomJoints(fingerJoints);
      fingerJoints.remove(0);

      for (OneDegreeOfFreedomJoint fingerJoint : fingerJoints)
      {
         double q = fingerJoint.getQYoVariable().getDoubleValue() * getFingerClosedJointAngleSign(robotSide);
         ret += q;
         if (DEBUG)
         {
            PrintTools.debug(this, fingerJoint.getName() + " q : " + q);
         }
      }

      return ret;
   }

   private HandDesiredConfigurationBehavior testHandDesiredConfigurationBehavior(HandDesiredConfigurationMessage handDesiredConfigurationMessage,
                                                                                 double trajectoryTime)
         throws SimulationExceededMaximumTimeException
   {
      final HandDesiredConfigurationBehavior behavior = new HandDesiredConfigurationBehavior(drcBehaviorTestHelper.getRobotName(), "test",
                                                                                             drcBehaviorTestHelper.getRos2Node(), drcBehaviorTestHelper.getYoTime());

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
