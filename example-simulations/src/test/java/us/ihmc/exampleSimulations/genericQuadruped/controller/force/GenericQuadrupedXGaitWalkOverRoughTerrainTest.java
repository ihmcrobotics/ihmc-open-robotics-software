package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import junit.framework.AssertionFailedError;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitWalkOverRoughTerrainTest;

import java.io.IOException;

@ContinuousIntegrationPlan(categories = IntegrationCategory.EXCLUDE)
public class GenericQuadrupedXGaitWalkOverRoughTerrainTest extends QuadrupedXGaitWalkOverRoughTerrainTest
{
   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 2000000)
   public void testWalkingOverTiledGround() throws IOException, AssertionFailedError
   {
      super.testWalkingOverTiledGround();
   }

   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 2000000)
   public void testWalkingOverSingleStepUp() throws IOException, AssertionFailedError
   {
      super.testWalkingOverSingleStepUp();
   }

   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 2000000)
   public void testWalkingOverConsecutiveRamps() throws IOException, AssertionFailedError
   {
      super.testWalkingOverConsecutiveRamps();
   }

   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 2000000)
   public void testWalkingOverCinderBlockField() throws IOException, AssertionFailedError
   {
      super.testWalkingOverCinderBlockField();
   }

   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
}
