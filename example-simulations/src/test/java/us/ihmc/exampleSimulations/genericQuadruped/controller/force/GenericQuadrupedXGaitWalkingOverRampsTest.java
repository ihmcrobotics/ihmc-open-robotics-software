package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedDefaultInitialPosition;
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
   
   @ContinuousIntegrationTest(estimatedDuration = 238.9)
   @Test(timeout = 1200000)
   public void testWalkingDownSlope() throws IOException
   {
      super.testWalkingDownSlope(new GenericQuadrupedDefaultInitialPosition());
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 431.3)
   @Test(timeout = 2200000)
   public void testWalkingOverShallowRamps() throws IOException
   {
      super.testWalkingOverShallowRamps(0.575);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 195.3)
   @Test(timeout = 980000)
   public void testWalkingUpSlope() throws IOException
   {
      super.testWalkingUpSlope(new GenericQuadrupedDefaultInitialPosition());
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 405.9)
   @Test(timeout = 2000000)
   public void testWalkingOverAggressiveRamps() throws IOException
   {
      super.testWalkingOverAggressiveRamps(0.575);
   }
}
