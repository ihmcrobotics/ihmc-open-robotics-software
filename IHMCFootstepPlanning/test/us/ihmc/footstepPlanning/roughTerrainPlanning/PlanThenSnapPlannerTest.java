package us.ihmc.footstepPlanning.roughTerrainPlanning;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.simplePlanners.PlanThenSnapPlanner;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class PlanThenSnapPlannerTest extends FootstepPlannerOnRoughTerrainTest
{
   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   private final PlanThenSnapPlanner planner = new PlanThenSnapPlanner(new TurnWalkTurnPlanner(), PlanningTestTools.createDefaultFootPolygons());

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testOnStaircase()
   {
      super.testOnStaircase(new Vector3d(), true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSimpleStepOnBox()
   {
      super.testSimpleStepOnBox();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRandomEnvironment()
   {
      super.testRandomEnvironment();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testPartialGaps()
   {
      super.testPartialGaps();
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
