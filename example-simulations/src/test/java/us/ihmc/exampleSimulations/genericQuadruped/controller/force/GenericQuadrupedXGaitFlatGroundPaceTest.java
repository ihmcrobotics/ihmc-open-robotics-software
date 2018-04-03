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
   @ContinuousIntegrationTest(estimatedDuration = 98.9)
   @Test(timeout = 490000)
   public void testPacingForwardFast()
   {
      super.testPacingForwardFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 124.4)
   @Test(timeout = 620000)
   public void testPacingForwardSlow()
   {
      super.testPacingForwardSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 89.5)
   @Test(timeout = 450000)
   public void testPacingBackwardsFast()
   {
      super.testPacingBackwardsFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 119.6)
   @Test(timeout = 600000)
   public void testPacingBackwardsSlow()
   {
      super.testPacingBackwardsSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 185.9)
   @Test(timeout = 930000)
   public void testPacingInAForwardLeftCircle()
   {
      super.testPacingInAForwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 138.7)
   @Test(timeout = 690000)
   public void testPacingInAForwardRightCircle()
   {
      super.testPacingInAForwardRightCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 134.0)
   @Test(timeout = 670000)
   public void testPacingInABackwardLeftCircle()
   {
      super.testPacingInABackwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 194.6)
   @Test(timeout = 970000)
   public void testPacingInABackwardRightCircle()
   {
      super.testPacingInABackwardRightCircle();
   }
}
