package us.ihmc.footstepPlanning.testTools;

import us.ihmc.footstepPlanning.FootstepPlanner;

public interface PlanningTest
{
   FootstepPlanner getPlanner();
   boolean visualize();
   boolean keepUp();
}
