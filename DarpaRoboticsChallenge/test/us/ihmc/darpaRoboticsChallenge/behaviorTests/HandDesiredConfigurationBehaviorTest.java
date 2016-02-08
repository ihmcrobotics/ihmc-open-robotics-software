package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandDesiredConfigurationBehavior;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;

public abstract class HandDesiredConfigurationBehaviorTest implements MultiRobotTestInterface
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
      
      

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(HandDesiredConfigurationBehaviorTest.class + " after class.");
   }

   private final boolean DEBUG = false;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @Before
   public void setUp()
   {
      DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();


      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
   }

   @DeployableTestMethod(estimatedDuration = 27.7)
   @Test(timeout = 83115)
   public void testCloseHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 2.0;

      double fingerJointQInitial = getTotalFingerJointQ(robotSide);
      HandDesiredConfigurationBehavior behavior = testHandDesiredConfigurationBehavior(new HandDesiredConfigurationMessage(robotSide, HandConfiguration.CLOSE), trajectoryTime);
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(behavior);
      assertTrue(success);
      double fingerJointQFinal = getTotalFingerJointQ(robotSide);

      PrintTools.debug(this, "fingerJointQInitial: " + fingerJointQInitial);
      PrintTools.debug(this, "fingerJointQFinal : " + fingerJointQFinal);


      assertTrue(fingerJointQFinal > fingerJointQInitial);
      assertTrue(behavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @DeployableTestMethod(estimatedDuration = 27.7)
   @Test(timeout = 83115)
   public void testStopCloseHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      PrintTools.debug(this, "Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 0.3; // [0.3] Hand closes quickly!
      double stopTime = trajectoryTime / 2.0;

      PrintTools.debug(this, "Initializing Behavior");
      HandDesiredConfigurationBehavior behavior = testHandDesiredConfigurationBehavior(new HandDesiredConfigurationMessage(robotSide, HandConfiguration.CLOSE), trajectoryTime);

      PrintTools.debug(this, "Starting Behavior");
      double fingerJointQInitial = getTotalFingerJointQ(robotSide);
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(behavior, stopTime);
      assertTrue(success);
      PrintTools.debug(this, "Stopping Behavior");
      double fingerJointQAtStop = getTotalFingerJointQ(robotSide);
      behavior.stop();
      assertTrue(!behavior.isDone());

      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(behavior, 1.0);
      assertTrue(success);
      double fingerJointQFinal = getTotalFingerJointQ(robotSide);

      PrintTools.debug(this, "fingerJointQInitial: " + fingerJointQInitial);
      PrintTools.debug(this, "fingerJointQAtStop : " + fingerJointQAtStop);
      PrintTools.debug(this, "fingerJointQFinal : " + fingerJointQFinal);

      assertTrue(Math.abs(fingerJointQFinal - fingerJointQAtStop) < 3.0);
      assertTrue(!behavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }
   
   @DeployableTestMethod(estimatedDuration = 27.7)
   @Test(timeout = 83115)
   public void testPauseAndResumeCloseHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      PrintTools.debug(this, "Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 0.3; // [0.3] Hand closes quickly!
      double stopTime = trajectoryTime / 2.0;

      PrintTools.debug(this, "Initializing Behavior");
      HandDesiredConfigurationBehavior behavior = testHandDesiredConfigurationBehavior(new HandDesiredConfigurationMessage(robotSide, HandConfiguration.CLOSE), trajectoryTime);

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

      BambooTools.reportTestFinishedMessage();
   }

   private double getTotalFingerJointQ(RobotSide robotSide)
   {
      double ret = 0.0;

      ArrayList<OneDegreeOfFreedomJoint> fingerJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      SDFJointNameMap jointNameMap = (SDFJointNameMap) drcBehaviorTestHelper.getSDFFullRobotModel().getRobotSpecificJointNames();
      Joint wristJoint = drcBehaviorTestHelper.getRobot().getJoint(jointNameMap.getJointBeforeHandName(robotSide));
      wristJoint.recursiveGetOneDegreeOfFreedomJoints(fingerJoints);
      fingerJoints.remove(0);

      for (OneDegreeOfFreedomJoint fingerJoint : fingerJoints)
      {
         double q = fingerJoint.getQ().getDoubleValue();
         ret += q;
         if (DEBUG)
         {
            PrintTools.debug(this, fingerJoint.getName() + " q : " + q);
         }
      }

      return ret;
   }

   private HandDesiredConfigurationBehavior testHandDesiredConfigurationBehavior(HandDesiredConfigurationMessage handDesiredConfigurationMessage, double trajectoryTime)
         throws SimulationExceededMaximumTimeException
   {
      final HandDesiredConfigurationBehavior behavior = new HandDesiredConfigurationBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getYoTime());

      behavior.initialize();
      behavior.setInput(handDesiredConfigurationMessage);
      assertTrue(behavior.hasInputBeenSet());

      return behavior;
   }
}
