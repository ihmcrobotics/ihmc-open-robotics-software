package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import junit.framework.AssertionFailedError;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedXGaitSettings;
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
      super.testWalkingOverTiledGround(new GenericQuadrupedXGaitSettings());
   }

   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 2000000)
   public void testWalkingOverSingleStepUp() throws IOException, AssertionFailedError
   {
      super.testWalkingOverSingleStepUp(new GenericQuadrupedXGaitSettings());
   }

   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 2000000)
   public void testWalkingOverConsecutiveRamps() throws IOException, AssertionFailedError
   {
      super.testWalkingOverConsecutiveRamps(new GenericQuadrupedXGaitSettings());
   }

   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 2000000)
   public void testWalkingOverCinderBlockField() throws IOException, AssertionFailedError
   {
      super.testWalkingOverCinderBlockField(new GenericQuadrupedXGaitSettings());
   }

   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
}
