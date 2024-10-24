package us.ihmc.avatar.behaviorTests;

import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.avatar.testTools.scs2.SCS2BehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HighLevelStateBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

@Disabled
@Deprecated
public abstract class DRCHighLevelStateBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());

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
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCHighLevelStateBehaviorTest.class + " after class.");
   }

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
   public void testWalkingState()
   {
      testState(HighLevelControllerName.WALKING);
   }

   @Test
   public void testDoNothingBahviourState()
   {
      testState(HighLevelControllerName.DO_NOTHING_BEHAVIOR);

      for (OneDoFJointBasics joint : behaviorTestHelper.getControllerFullRobotModel().getControllableOneDoFJoints())
      {
         String jointName = joint.getName();

         if (!jointName.contains("hokuyo"))
         {
            assertTrue(joint.getName() + " tau : " + joint.getTau(), joint.getTau() == 0.0);
         }
      }
   }

   @Test
   public void testDiagnosticsState()
   {
      testState(HighLevelControllerName.DIAGNOSTICS);
   }

   private void testState(HighLevelControllerName desiredState)
   {
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);

      double trajectoryTime = 2.0;

      final HighLevelStateBehavior highLevelStateBehavior = new HighLevelStateBehavior(behaviorTestHelper.getRobotName(), behaviorTestHelper.getROS2Node());

      highLevelStateBehavior.initialize();
      highLevelStateBehavior.setInput(HumanoidMessageTools.createHighLevelStateMessage(desiredState));
      assertTrue(highLevelStateBehavior.hasInputBeenSet());

      success = behaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(highLevelStateBehavior, trajectoryTime);
      assertTrue(success);

      HighLevelControllerName actualState = getCurrentHighLevelState();

      assertTrue(highLevelStateBehavior.isDone());
      assertTrue("Actual high level state: " + actualState + ", does not match desired high level state: " + desiredState + ".",
                 desiredState.equals(actualState));
   }

   private HighLevelControllerName getCurrentHighLevelState()
   {
      return behaviorTestHelper.getHighLevelHumanoidControllerFactory().getCurrentHighLevelControlState();
   }
}
