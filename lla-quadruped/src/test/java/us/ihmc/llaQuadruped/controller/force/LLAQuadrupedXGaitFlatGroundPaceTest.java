package us.ihmc.llaQuadruped.controller.force;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitFlatGroundPaceTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class LLAQuadrupedXGaitFlatGroundPaceTest extends QuadrupedXGaitFlatGroundPaceTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 11.1)
   @Test(timeout = 55000)
   public void testPacingForwardFast()
   {
      super.testPacingForwardFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 13.5)
   @Test(timeout = 68000)
   public void testPacingForwardSlow()
   {
      super.testPacingForwardSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 12.7)
   @Test(timeout = 63000)
   public void testPacingBackwardsFast()
   {
      super.testPacingBackwardsFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 16.8)
   @Test(timeout = 84000)
   public void testPacingBackwardsSlow()
   {
      super.testPacingBackwardsSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 26.4)
   @Test(timeout = 130000)
   public void testPacingInAForwardLeftCircle()
   {
      super.testPacingInAForwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 24.1)
   @Test(timeout = 120000)
   public void testPacingInAForwardRightCircle()
   {
      super.testPacingInAForwardRightCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.8)
   @Test(timeout = 130000)
   public void testPacingInABackwardLeftCircle()
   {
      super.testPacingInABackwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 22.9)
   @Test(timeout = 110000)
   public void testPacingInABackwardRightCircle()
   {
      super.testPacingInABackwardRightCircle();
   }
}
