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
   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testWalkingForwardFast()
   {
      super.testWalkingForwardFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test(timeout = 300000)
   public void testWalkingForwardSlow()
   {
      super.testWalkingForwardSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testWalkingBackwardsFast()
   {
      super.testWalkingBackwardsFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 300000)
   public void testWalkingBackwardsSlow()
   {
      super.testWalkingBackwardsSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 70.0)
   @Test(timeout = 300000)
   public void testWalkingInAForwardLeftCircle()
   {
      super.testWalkingInAForwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 70.0)
   @Test(timeout = 300000)
   public void testWalkingInAForwardRightCircle()
   {
      super.testWalkingInAForwardRightCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 75.0)
   @Test(timeout = 300000)
   public void testWalkingInABackwardLeftCircle()
   {
      super.testWalkingInABackwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 70.0)
   @Test(timeout = 300000)
   public void testWalkingInABackwardRightCircle()
   {
      super.testWalkingInABackwardRightCircle();
   }
}
