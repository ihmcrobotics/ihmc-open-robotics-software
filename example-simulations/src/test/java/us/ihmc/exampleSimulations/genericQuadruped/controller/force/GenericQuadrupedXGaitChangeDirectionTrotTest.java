package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitChangeDirectionTest;

public class GenericQuadrupedXGaitChangeDirectionTrotTest extends QuadrupedXGaitChangeDirectionTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testTrottingForwardThenChangeToLeft()
   {
      super.testTrottingForwardThenChangeToLeft();
   }

   @ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testTrottingBackwardThenChangeToLeft()
   {
      super.testTrottingBackwardThenChangeToLeft();
   }

   @ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testTrottingForwardThenChangeToRight()
   {
      super.testTrottingForwardThenChangeToRight();
   }

   @ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testTrottingBackwardThenChangeToRight()
   {
      super.testTrottingBackwardThenChangeToRight();
   }
}