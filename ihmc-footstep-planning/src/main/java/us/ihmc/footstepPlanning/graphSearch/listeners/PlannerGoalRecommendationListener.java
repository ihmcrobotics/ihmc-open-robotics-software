package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.footstepPlanning.FootstepPlannerGoal;

public interface PlannerGoalRecommendationListener
{
   void notifyWithPlannerGoalRecommendation(FootstepPlannerGoal footstepPlannerGoal);
}
