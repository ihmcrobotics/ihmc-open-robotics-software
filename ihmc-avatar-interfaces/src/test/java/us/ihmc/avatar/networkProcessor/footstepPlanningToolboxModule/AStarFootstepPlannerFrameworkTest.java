package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import us.ihmc.footstepPlanning.FootstepPlannerType;

public class AStarFootstepPlannerFrameworkTest extends NetworkedFootstepPlannerFrameworkTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }
}
