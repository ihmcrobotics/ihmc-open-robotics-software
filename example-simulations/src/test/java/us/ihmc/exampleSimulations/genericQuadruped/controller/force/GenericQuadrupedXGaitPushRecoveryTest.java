package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitFlatGroundWalkingTest;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitPushRecoveryTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class GenericQuadrupedXGaitPushRecoveryTest extends QuadrupedXGaitPushRecoveryTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 630000)
   public void testWalkingForwardFastWithPush()
   {
      super.testWalkingWithPush(90.0, 0.8);
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 630000)
   public void testScriptedWalkingForwardFastWithPush()
   {
      super.testScriptedWalkingWithPush(90.0, 0.8, 0.35);
   }
}
