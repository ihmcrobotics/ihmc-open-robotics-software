package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitTurnTest;

public class GenericQuadrupedXGaitTurnPaceTest extends QuadrupedXGaitTurnTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testPacingForwardThenTurnLeft() { super.testPacingForwardThenTurnLeft(); }

   @ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testPacingBackwardThenTurnLeft() { super.testPacingBackwardThenTurnLeft(); }

   @ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testPacingForwardThenTurnRight() { super.testPacingForwardThenTurnRight(); }

   @ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testPacingBackwardThenTurnRight() { super.testPacingBackwardThenTurnRight(); }



   @ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testPacingDiagonallyForwardLeft() { super.testPacingDiagonallyForwardLeft(); }

   @ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testPacingDiagonallyBackwardLeft() { super.testPacingDiagonallyBackwardLeft(); }

   @ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testPacingDiagonallyForwardRight() { super.testPacingDiagonallyForwardRight(); }

   @ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testPacingDiagonallyBackwardRight() { super.testPacingDiagonallyBackwardRight(); }


}