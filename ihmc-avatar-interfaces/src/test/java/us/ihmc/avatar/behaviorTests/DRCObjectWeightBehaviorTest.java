package us.ihmc.avatar.behaviorTests;

import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.avatar.testTools.scs2.SCS2BehaviorTestHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ObjectWeightBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class DRCObjectWeightBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double epsilon = 10e-8;
   private SCS2BehaviorTestHelper behaviorTestHelper;

   @BeforeEach
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DefaultCommonAvatarEnvironment testEnvironment = new DefaultCommonAvatarEnvironment();
      SCS2AvatarTestingSimulation simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                                                        testEnvironment,
                                                                                                                        simulationTestingParameters);
      simulationTestHelper.start();
      behaviorTestHelper = new SCS2BehaviorTestHelper(simulationTestHelper);
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

   @Test
   public void testConstructorAndSetInput()
   {
      ObjectWeightBehavior behavior = new ObjectWeightBehavior(behaviorTestHelper.getRobotName(), behaviorTestHelper.getROS2Node());
      behavior.setInput(HumanoidMessageTools.createObjectWeightPacket(RobotSide.LEFT, 0.0));
      assertTrue(behavior.hasInputBeenSet());
   }

   @Disabled("Needs to be reimplemented")
   @Test
   public void testSettingWeight()
   {
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);

      ObjectWeightBehavior objectWeightBehavior = new ObjectWeightBehavior(behaviorTestHelper.getRobotName(), behaviorTestHelper.getROS2Node());
      YoDouble rightMass = (YoDouble) behaviorTestHelper.findVariable("rightTool", "rightToolObjectMass");
      YoDouble leftMass = (YoDouble) behaviorTestHelper.findVariable("leftTool", "leftToolObjectMass");

      double weightLeft = 1.5;
      objectWeightBehavior.initialize();
      objectWeightBehavior.setInput(HumanoidMessageTools.createObjectWeightPacket(RobotSide.LEFT, weightLeft));
      success = behaviorTestHelper.executeBehaviorUntilDone(objectWeightBehavior);
      assertTrue(success);
      assertTrue(MathTools.epsilonEquals(leftMass.getDoubleValue(), weightLeft, epsilon));
      assertTrue(MathTools.epsilonEquals(rightMass.getDoubleValue(), 0.0, epsilon));

      double weightRight = 0.8;
      objectWeightBehavior.initialize();
      objectWeightBehavior.setInput(HumanoidMessageTools.createObjectWeightPacket(RobotSide.RIGHT, weightRight));
      success = behaviorTestHelper.executeBehaviorUntilDone(objectWeightBehavior);
      assertTrue(success);
      assertTrue(MathTools.epsilonEquals(leftMass.getDoubleValue(), weightLeft, epsilon));
      assertTrue(MathTools.epsilonEquals(rightMass.getDoubleValue(), weightRight, epsilon));
   }
}
