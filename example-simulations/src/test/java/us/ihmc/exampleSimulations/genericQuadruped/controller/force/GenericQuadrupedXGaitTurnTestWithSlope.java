package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitTurnTestWithSlope;

import java.io.IOException;

public class GenericQuadrupedXGaitTurnTestWithSlope extends QuadrupedXGaitTurnTestWithSlope
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testTrottingForwardThenTurnLeftWithSlope() throws IOException
   {
      super.testTrottingForwardThenTurnLeftWithSlope();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testTrottingBackwardThenTurnLeftWithSlope()throws IOException
   {
      super.testTrottingBackwardThenTurnLeftWithSlope();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testTrottingForwardThenTurnRightWithSlope()throws IOException
   {
      super.testTrottingForwardThenTurnRightWithSlope();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testTrottingBackwardThenTurnRightWithSlope()throws IOException
   {
      super.testTrottingBackwardThenTurnRightWithSlope();
   }



   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testTrottingDiagonallyForwardLeftWithSlope()throws IOException
   {
      super.testTrottingDiagonallyForwardLeftWithSlope();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testTrottingDiagonallyBackwardLeftWithSlope()throws IOException
   {
      super.testTrottingDiagonallyBackwardLeftWithSlope();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testTrottingDiagonallyForwardRightWithSlope()throws IOException
   {
      super.testTrottingDiagonallyForwardRightWithSlope();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testTrottingDiagonallyBackwardRightWithSlope()throws IOException
   {
      super.testTrottingDiagonallyBackwardRightWithSlope();
   }
}