package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.planning.QuadrupedBodyPathPlanTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class GenericQuadrupedBodyPathPlanTest extends QuadrupedBodyPathPlanTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Test(timeout = 200000)
   @ContinuousIntegrationTest(estimatedDuration = 100)
   @Override
   public void testSimpleBodyPathPlan()
   {
      super.testSimpleBodyPathPlan();
   }

   @Test(timeout = 200000)
   @ContinuousIntegrationTest(estimatedDuration = 120)
   @Override
   public void testBodyPathAroundASimpleMaze()
   {
      super.testBodyPathAroundASimpleMaze();
   }
}
