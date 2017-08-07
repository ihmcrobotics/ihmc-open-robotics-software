package us.ihmc.llaQuadruped.controller.force;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitFlatGroundTrotTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class LLAQuadrupedXGaitFlatGroundTrotTest extends QuadrupedXGaitFlatGroundTrotTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 11.1)
   @Test(timeout = 55000)
   public void testTrottingForwardFast()
   {
      super.testTrottingForwardFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 13.5)
   @Test(timeout = 68000)
   public void testTrottingForwardSlow()
   {
      super.testTrottingForwardSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 12.7)
   @Test(timeout = 63000)
   public void testTrottingBackwardsFast()
   {
      super.testTrottingBackwardsFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 16.8)
   @Test(timeout = 84000)
   public void testTrottingBackwardsSlow()
   {
      super.testTrottingBackwardsSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 26.4)
   @Test(timeout = 130000)
   public void testTrottingInAForwardLeftCircle()
   {
      super.testTrottingInAForwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 24.1)
   @Test(timeout = 120000)
   public void testTrottingInAForwardRightCircle()
   {
      super.testTrottingInAForwardRightCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.8)
   @Test(timeout = 130000)
   public void testTrottingInABackwardLeftCircle()
   {
      super.testTrottingInABackwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 22.9)
   @Test(timeout = 110000)
   public void testTrottingInABackwardRightCircle()
   {
      super.testTrottingInABackwardRightCircle();
   }
}
