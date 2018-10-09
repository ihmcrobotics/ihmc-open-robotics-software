package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import us.ihmc.footstepPlanning.FootstepPlannerType;

public class VisGraphAStarPlannerToolboxTest extends FootstepPlannerToolboxTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.VIS_GRAPH_WITH_A_STAR;
   }
}
