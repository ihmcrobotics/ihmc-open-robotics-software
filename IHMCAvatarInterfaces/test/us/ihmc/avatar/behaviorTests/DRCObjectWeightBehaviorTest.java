package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ObjectWeightBehavior;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ObjectWeightPacket;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCObjectWeightBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
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
   
   @ContinuousIntegrationTest(estimatedDuration = 14.6)
   @Test(timeout = 73000)
   public void testConstructorAndSetInput()
   {
      ObjectWeightBehavior behavior = new ObjectWeightBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge());
      behavior.setInput(new ObjectWeightPacket(RobotSide.LEFT, 0.0));
      assertTrue(behavior.hasInputBeenSet());
   }
   
   @Ignore("Needs to be reimplemented")
   @ContinuousIntegrationTest(estimatedDuration = 19.6)
   @Test(timeout = 98000)
   public void testSettingWeight() throws SimulationExceededMaximumTimeException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      
      ObjectWeightBehavior objectWeightBehavior = new ObjectWeightBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge());
      DoubleYoVariable rightMass = (DoubleYoVariable) drcBehaviorTestHelper.getSimulationConstructionSet().getVariable("rightTool", "rightToolObjectMass");
      DoubleYoVariable leftMass = (DoubleYoVariable) drcBehaviorTestHelper.getSimulationConstructionSet().getVariable("leftTool", "leftToolObjectMass");
      
      double weightLeft = 1.5;
      objectWeightBehavior.initialize();
      objectWeightBehavior.setInput(new ObjectWeightPacket(RobotSide.LEFT, weightLeft));
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(objectWeightBehavior);
      assertTrue(success);
      assertTrue(MathTools.epsilonEquals(leftMass.getDoubleValue(), weightLeft, epsilon));
      assertTrue(MathTools.epsilonEquals(rightMass.getDoubleValue(), 0.0, epsilon));
      
      double weightRight = 0.8;
      objectWeightBehavior.initialize();
      objectWeightBehavior.setInput(new ObjectWeightPacket(RobotSide.RIGHT, weightRight));
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(objectWeightBehavior);
      assertTrue(success);
      assertTrue(MathTools.epsilonEquals(leftMass.getDoubleValue(), weightLeft, epsilon));
      assertTrue(MathTools.epsilonEquals(rightMass.getDoubleValue(), weightRight, epsilon));
   }
}
