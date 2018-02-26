package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ObjectWeightBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ObjectWeightPacket;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class DRCObjectWeightBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double epsilon = 10e-8;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DefaultCommonAvatarEnvironment testEnvironment = new DefaultCommonAvatarEnvironment();
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(),
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
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
         drcBehaviorTestHelper.destroySimulation();
         drcBehaviorTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void testConstructorAndSetInput()
   {
      ObjectWeightBehavior behavior = new ObjectWeightBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge());
      behavior.setInput(HumanoidMessageTools.createObjectWeightPacket(RobotSide.LEFT, 0.0));
      assertTrue(behavior.hasInputBeenSet());
   }

   public void testSettingWeight() throws SimulationExceededMaximumTimeException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      ObjectWeightBehavior objectWeightBehavior = new ObjectWeightBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge());
      YoDouble rightMass = (YoDouble) drcBehaviorTestHelper.getSimulationConstructionSet().getVariable("rightTool", "rightToolObjectMass");
      YoDouble leftMass = (YoDouble) drcBehaviorTestHelper.getSimulationConstructionSet().getVariable("leftTool", "leftToolObjectMass");

      double weightLeft = 1.5;
      objectWeightBehavior.initialize();
      objectWeightBehavior.setInput(HumanoidMessageTools.createObjectWeightPacket(RobotSide.LEFT, weightLeft));
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(objectWeightBehavior);
      assertTrue(success);
      assertTrue(MathTools.epsilonEquals(leftMass.getDoubleValue(), weightLeft, epsilon));
      assertTrue(MathTools.epsilonEquals(rightMass.getDoubleValue(), 0.0, epsilon));

      double weightRight = 0.8;
      objectWeightBehavior.initialize();
      objectWeightBehavior.setInput(HumanoidMessageTools.createObjectWeightPacket(RobotSide.RIGHT, weightRight));
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(objectWeightBehavior);
      assertTrue(success);
      assertTrue(MathTools.epsilonEquals(leftMass.getDoubleValue(), weightLeft, epsilon));
      assertTrue(MathTools.epsilonEquals(rightMass.getDoubleValue(), weightRight, epsilon));
   }
}
