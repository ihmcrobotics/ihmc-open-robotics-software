package us.ihmc.llaQuadruped.controller.force;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitFlatGroundWalkingTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class LLAQuadrupedXGaitFlatGroundWalkingTest extends QuadrupedXGaitFlatGroundWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testWalkingForwardFast()
   {
      super.testWalkingForwardFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testWalkingForwardSlow()
   {
      super.testWalkingForwardSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testWalkingBackwardsFast()
   {
      super.testWalkingBackwardsFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testWalkingBackwardsSlow()
   {
      super.testWalkingBackwardsSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testWalkingInAForwardLeftCircle()
   {
      super.testWalkingInAForwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testWalkingInAForwardRightCircle()
   {
      super.testWalkingInAForwardRightCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testWalkingInABackwardLeftCircle()
   {
      super.testWalkingInABackwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testWalkingInABackwardRightCircle()
   {
      super.testWalkingInABackwardRightCircle();
   }
}
