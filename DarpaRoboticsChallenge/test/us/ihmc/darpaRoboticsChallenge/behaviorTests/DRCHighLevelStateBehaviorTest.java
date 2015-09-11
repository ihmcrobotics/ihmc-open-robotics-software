package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.communication.packets.HighLevelStatePacket;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HighLevelStateBehavior;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.utilities.io.printing.PrintTools;

public abstract class DRCHighLevelStateBehaviorTest implements MultiRobotTestInterface
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
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCHighLevelStateBehaviorTest.class + " after class.");
   }

   private static final boolean DEBUG = false;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @Before
   public void setUp()
   {
      DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
   }

   @DeployableTestMethod(estimatedDuration = 21.5)
   @Test(timeout = 64580)
   public void testDoNothingBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double trajectoryTime = 2.0;
      HighLevelState desiredState = HighLevelState.DO_NOTHING_BEHAVIOR;

      final HighLevelStateBehavior highLevelStateBehavior = new HighLevelStateBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge());

      highLevelStateBehavior.initialize();
      highLevelStateBehavior.setInput(new HighLevelStatePacket(desiredState));
      assertTrue(highLevelStateBehavior.hasInputBeenSet());

      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(highLevelStateBehavior, trajectoryTime);
      assertTrue(success);

      assertTrue(highLevelStateBehavior.isDone());

      OneDegreeOfFreedomJoint[] oneDofJoints = drcBehaviorTestHelper.getRobot().getOneDoFJoints();

      for (OneDegreeOfFreedomJoint joint : oneDofJoints)
      {
         String jointName = joint.getName();
         double tau = joint.getTau().getDoubleValue();
         PrintTools.debug(this, joint.getName() + " tau : " + tau);

         if (!jointName.contains("hokuyo"))
         {
            assertTrue(tau == 0.0);
         }
      }
      
      HighLevelState actualState = getCurrentHighLevelState();

      assertTrue("Actual high level state: " + actualState + ", does not match desired high level state: " + desiredState + ".",
            desiredState.equals(actualState));
      
      BambooTools.reportTestFinishedMessage();
   }

   @DeployableTestMethod(estimatedDuration = 21.5)
   @Test(timeout = 64580)
   public void testRandomState() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double trajectoryTime = 2.0;
      HighLevelState[] highLevelStates = HighLevelState.values();
      HighLevelState desiredState = highLevelStates[RandomTools.generateRandomInt(new Random(), 0, highLevelStates.length - 1)];

      final HighLevelStateBehavior highLevelStateBehavior = new HighLevelStateBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge());

      highLevelStateBehavior.initialize();
      highLevelStateBehavior.setInput(new HighLevelStatePacket(desiredState));
      assertTrue(highLevelStateBehavior.hasInputBeenSet());

      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(highLevelStateBehavior, trajectoryTime);
      assertTrue(success);

      HighLevelState actualState = getCurrentHighLevelState();

      assertTrue(highLevelStateBehavior.isDone());
      assertTrue("Actual high level state: " + actualState + ", does not match desired high level state: " + desiredState + ".",
            desiredState.equals(actualState));

      BambooTools.reportTestFinishedMessage();
   }

   private HighLevelState getCurrentHighLevelState()
   {
      return drcBehaviorTestHelper.getDRCSimulationFactory().getControllerFactory().getHighLevelHumanoidControllerManager().getCurrentHighLevelState();
   }
}
