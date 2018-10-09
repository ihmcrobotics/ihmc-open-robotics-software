package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import us.ihmc.footstepPlanning.FootstepPlannerType;

public class PlanThenSnapPlannerToolboxTest extends FootstepPlannerToolboxTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.PLAN_THEN_SNAP;
   }
}
