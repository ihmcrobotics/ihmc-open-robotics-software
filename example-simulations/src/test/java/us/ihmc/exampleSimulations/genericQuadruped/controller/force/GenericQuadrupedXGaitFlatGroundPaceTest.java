package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitFlatGroundPaceTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class GenericQuadrupedXGaitFlatGroundPaceTest extends QuadrupedXGaitFlatGroundPaceTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 11.1)
   @Test(timeout = 80000)
   public void testPacingForwardFast()
   {
      super.testPacingForwardFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 13.5)
   @Test(timeout = 75000)
   public void testPacingForwardSlow()
   {
      super.testPacingForwardSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 12.7)
   @Test(timeout = 100000)
   public void testPacingBackwardsFast()
   {
      super.testPacingBackwardsFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 16.8)
   @Test(timeout = 100000)
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
