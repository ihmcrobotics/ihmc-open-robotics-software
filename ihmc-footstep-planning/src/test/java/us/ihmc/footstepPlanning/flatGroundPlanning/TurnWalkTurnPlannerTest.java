package us.ihmc.footstepPlanning.flatGroundPlanning;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class TurnWalkTurnPlannerTest extends FootstepPlannerOnFlatGroundTest
{
   private static final boolean visualize = false;
   private final TurnWalkTurnPlanner planner = new TurnWalkTurnPlanner();

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testJustStraightLine()
   {
      super.testJustStraightLine();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testStraightLineWithInitialTurn()
   {
      super.testStraightLineWithInitialTurn();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testJustTurnInPlace()
   {
      super.testJustTurnInPlace();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testRandomPoses()
   {
      super.testRandomPoses();
   }

   @Override
   public FootstepPlanner getPlanner()
   {
      return planner;
   }

   @Override
   public boolean visualize()
   {
      return visualize;
   }
}
