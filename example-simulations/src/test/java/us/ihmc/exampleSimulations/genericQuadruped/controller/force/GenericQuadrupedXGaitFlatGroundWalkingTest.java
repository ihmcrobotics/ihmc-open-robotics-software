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
   
   @ContinuousIntegrationTest(estimatedDuration = 125.8)
   @Test(timeout = 630000)
   public void testWalkingForwardFast()
   {
      super.testFlatGroundWalking(0.8);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 144.6)
   @Test(timeout = 720000)
   public void testWalkingForwardSlow()
   {
      super.testFlatGroundWalking(0.1);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 91.4)
   @Test(timeout = 460000)
   public void testWalkingBackwardsFast()
   {
      super.testFlatGroundWalking(-0.8);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 133.1)
   @Test(timeout = 670000)
   public void testWalkingBackwardsSlow()
   {
      super.testFlatGroundWalking(-0.1);
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 221.0)
   @Test(timeout = 1100000)
   public void testWalkingInAForwardLeftCircle()
   {
      super.testWalkingInAForwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 246.9)
   @Test(timeout = 1200000)
   public void testWalkingInAForwardRightCircle()
   {
      super.testWalkingInAForwardRightCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 236.1)
   @Test(timeout = 1200000)
   public void testWalkingInABackwardLeftCircle()
   {
      super.testWalkingInABackwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 308.4)
   @Test(timeout = 1500000)
   public void testWalkingInABackwardRightCircle()
   {
      super.testWalkingInABackwardRightCircle();
   }
}
