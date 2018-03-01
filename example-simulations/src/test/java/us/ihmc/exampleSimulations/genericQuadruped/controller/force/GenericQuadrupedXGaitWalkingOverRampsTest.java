package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitWalkingOverRampsTest;

import java.io.IOException;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class GenericQuadrupedXGaitWalkingOverRampsTest extends QuadrupedXGaitWalkingOverRampsTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 600000)
   public void testWalkingDownSlope() throws IOException
   {
      super.testWalkingDownSlope();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 140.0)
   @Test(timeout = 500000)
   public void testWalkingOverShallowRamps() throws IOException
   {
      super.testWalkingOverShallowRamps();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 300000)
   public void testWalkingUpSlope() throws IOException
   {
      super.testWalkingUpSlope();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 300000)
   public void testWalkingOverAggressiveRamps() throws IOException
   {
      super.testWalkingUpSlope();
   }
}
