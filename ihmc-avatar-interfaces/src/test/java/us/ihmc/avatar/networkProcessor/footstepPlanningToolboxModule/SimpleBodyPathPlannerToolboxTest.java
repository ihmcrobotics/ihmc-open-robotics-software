package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import us.ihmc.footstepPlanning.FootstepPlannerType;

public class SimpleBodyPathPlannerToolboxTest extends FootstepPlannerToolboxTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.SIMPLE_BODY_PATH;
   }
}
