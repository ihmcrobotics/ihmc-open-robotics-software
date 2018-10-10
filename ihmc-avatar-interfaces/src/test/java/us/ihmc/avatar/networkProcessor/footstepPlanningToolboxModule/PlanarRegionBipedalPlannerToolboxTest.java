package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.SLOW)
public class PlanarRegionBipedalPlannerToolboxTest extends FootstepPlannerToolboxTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.PLANAR_REGION_BIPEDAL;
   }
}
