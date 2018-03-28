package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitFlatGroundWalkingTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class GenericQuadrupedXGaitFlatGroundWalkingTest extends QuadrupedXGaitFlatGroundWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 84.5)
   @Test(timeout = 420000)
   public void testWalkingForwardFast()
   {
      super.testWalkingForwardFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 88.4)
   @Test(timeout = 440000)
   public void testWalkingForwardSlow()
   {
      super.testWalkingForwardSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 83.5)
   @Test(timeout = 420000)
   public void testWalkingBackwardsFast()
   {
      super.testWalkingBackwardsFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 109.9)
   @Test(timeout = 550000)
   public void testWalkingBackwardsSlow()
   {
      super.testWalkingBackwardsSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 158.5)
   @Test(timeout = 790000)
   public void testWalkingInAForwardLeftCircle()
   {
      super.testWalkingInAForwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 165.4)
   @Test(timeout = 830000)
   public void testWalkingInAForwardRightCircle()
   {
      super.testWalkingInAForwardRightCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 161.0)
   @Test(timeout = 810000)
   public void testWalkingInABackwardLeftCircle()
   {
      super.testWalkingInABackwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 170.5)
   @Test(timeout = 850000)
   public void testWalkingInABackwardRightCircle()
   {
      super.testWalkingInABackwardRightCircle();
   }
}
