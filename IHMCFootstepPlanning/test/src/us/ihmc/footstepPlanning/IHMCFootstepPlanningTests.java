package us.ihmc.footstepPlanning;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.continuousIntegration.IntegrationCategory;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.FAST)
@SuiteClasses
({
   us.ihmc.footstepPlanning.flatGroundPlanning.BipedalFootstepPlannerNodeTest.class,
   us.ihmc.footstepPlanning.flatGroundPlanning.TurnWalkTurnPlannerTest.class,
   us.ihmc.footstepPlanning.graphSearch.PlanarRegionBaseOfCliffAvoiderTest.class,
   us.ihmc.footstepPlanning.graphSearch.PlanarRegionPotentialNextStepCalculatorTest.class,
   us.ihmc.footstepPlanning.graphSearch.SimplePlanarRegionBipedalAnytimeFootstepPlannerTest.class,
   us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionPolygonSnapperTest.class,
   us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapperTest.class,
   us.ihmc.footstepPlanning.polygonWiggling.PolygonWigglingTest.class,
   us.ihmc.footstepPlanning.roughTerrainPlanning.PlanarRegionBipedalFootstepPlannerOnFlatTest.class,
   us.ihmc.footstepPlanning.roughTerrainPlanning.PlanarRegionBipedalFootstepPlannerTest.class,
   us.ihmc.footstepPlanning.roughTerrainPlanning.PlanThenSnapPlannerTest.class,
   us.ihmc.footstepPlanning.scoring.BipedalStepAdjustmentCostCalculatorTest.class
})

public class IHMCFootstepPlanningTests
{
   public static void main(String[] args)
   {

   }
}
