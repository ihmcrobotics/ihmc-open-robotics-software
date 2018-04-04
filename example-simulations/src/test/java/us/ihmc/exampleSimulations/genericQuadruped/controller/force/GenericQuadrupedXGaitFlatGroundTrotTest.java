package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitFlatGroundTrotTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class GenericQuadrupedXGaitFlatGroundTrotTest extends QuadrupedXGaitFlatGroundTrotTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 126.3)
   @Test(timeout = 630000)
   public void testTrottingForwardFast()
   {
      super.testTrottingForwardFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 125.5)
   @Test(timeout = 630000)
   public void testTrottingForwardSlow()
   {
      super.testTrottingForwardSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 146.7)
   @Test(timeout = 730000)
   public void testTrottingBackwardsFast()
   {
      super.testTrottingBackwardsFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 154.3)
   @Test(timeout = 770000)
   public void testTrottingBackwardsSlow()
   {
      super.testTrottingBackwardsSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 253.9)
   @Test(timeout = 1300000)
   public void testTrottingInAForwardLeftCircle()
   {
      super.testTrottingInAForwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 367.4)
   @Test(timeout = 1800000)
   public void testTrottingInAForwardRightCircle()
   {
      super.testTrottingInAForwardRightCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 357.1)
   @Test(timeout = 1800000)
   public void testTrottingInABackwardLeftCircle()
   {
      super.testTrottingInABackwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 228.3)
   @Test(timeout = 1100000)
   public void testTrottingInABackwardRightCircle()
   {
      super.testTrottingInABackwardRightCircle();
   }
}
