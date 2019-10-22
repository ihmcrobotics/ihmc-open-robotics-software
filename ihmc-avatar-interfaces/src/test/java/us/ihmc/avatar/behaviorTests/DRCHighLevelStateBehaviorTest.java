package us.ihmc.avatar.behaviorTests;

import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HighLevelStateBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class DRCHighLevelStateBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());

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
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCHighLevelStateBehaviorTest.class + " after class.");
   }

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @BeforeEach
   public void setUp()
   {
      DefaultCommonAvatarEnvironment testEnvironment = new DefaultCommonAvatarEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), DRCObstacleCourseStartingLocation.DEFAULT,
                                                        simulationTestingParameters, getRobotModel());
   }

   @Test
   public void testWalkingState() throws SimulationExceededMaximumTimeException
   {
      testState(HighLevelControllerName.WALKING);
   }

   @Test
   public void testDoNothingBahviourState() throws SimulationExceededMaximumTimeException
   {
      testState(HighLevelControllerName.DO_NOTHING_BEHAVIOR);

      OneDegreeOfFreedomJoint[] oneDofJoints = drcBehaviorTestHelper.getRobot().getOneDegreeOfFreedomJoints();

      for (OneDegreeOfFreedomJoint joint : oneDofJoints)
      {
         String jointName = joint.getName();
         double tau = joint.getTauYoVariable().getDoubleValue();

         if (!jointName.contains("hokuyo"))
         {
            assertTrue(joint.getName() + " tau : " + tau, tau == 0.0);
         }
      }
   }

   @Test
   public void testDiagnosticsState() throws SimulationExceededMaximumTimeException
   {
      testState(HighLevelControllerName.DIAGNOSTICS);
   }

   private void testState(HighLevelControllerName desiredState) throws SimulationExceededMaximumTimeException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double trajectoryTime = 2.0;

      final HighLevelStateBehavior highLevelStateBehavior = new HighLevelStateBehavior(drcBehaviorTestHelper.getRobotName(), drcBehaviorTestHelper.getRos2Node());

      highLevelStateBehavior.initialize();
      highLevelStateBehavior.setInput(HumanoidMessageTools.createHighLevelStateMessage(desiredState));
      assertTrue(highLevelStateBehavior.hasInputBeenSet());

      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(highLevelStateBehavior, trajectoryTime);
      assertTrue(success);

      HighLevelControllerName actualState = getCurrentHighLevelState();

      assertTrue(highLevelStateBehavior.isDone());
      assertTrue("Actual high level state: " + actualState + ", does not match desired high level state: " + desiredState + ".",
                 desiredState.equals(actualState));
   }

   private HighLevelControllerName getCurrentHighLevelState()
   {
      return drcBehaviorTestHelper.getAvatarSimulation().getHighLevelHumanoidControllerFactory().getCurrentHighLevelControlState();
   }
}
