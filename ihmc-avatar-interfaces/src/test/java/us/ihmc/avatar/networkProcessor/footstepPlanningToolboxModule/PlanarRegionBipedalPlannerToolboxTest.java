package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import us.ihmc.footstepPlanning.FootstepPlannerType;

public class PlanarRegionBipedalPlannerToolboxTest extends FootstepPlannerToolboxTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.PLANAR_REGION_BIPEDAL;
   }
}
