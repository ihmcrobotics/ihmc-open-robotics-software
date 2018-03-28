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
   @ContinuousIntegrationTest(estimatedDuration = 84.6)
   @Test(timeout = 420000)
   public void testPacingForwardFast()
   {
      super.testPacingForwardFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 97.6)
   @Test(timeout = 490000)
   public void testPacingForwardSlow()
   {
      super.testPacingForwardSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 88.0)
   @Test(timeout = 440000)
   public void testPacingBackwardsFast()
   {
      super.testPacingBackwardsFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 106.6)
   @Test(timeout = 530000)
   public void testPacingBackwardsSlow()
   {
      super.testPacingBackwardsSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 134.0)
   @Test(timeout = 670000)
   public void testPacingInAForwardLeftCircle()
   {
      super.testPacingInAForwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 132.0)
   @Test(timeout = 660000)
   public void testPacingInAForwardRightCircle()
   {
      super.testPacingInAForwardRightCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 128.8)
   @Test(timeout = 640000)
   public void testPacingInABackwardLeftCircle()
   {
      super.testPacingInABackwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 132.8)
   @Test(timeout = 660000)
   public void testPacingInABackwardRightCircle()
   {
      super.testPacingInABackwardRightCircle();
   }
}
