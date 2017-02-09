package us.ihmc.footstepPlanning.testTools;

import us.ihmc.footstepPlanning.FootstepPlanner;

public interface PlanningTest
{
   abstract public FootstepPlanner getPlanner();
   abstract public boolean visualize();
}
