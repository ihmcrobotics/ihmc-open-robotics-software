package us.ihmc.avatar.footstepPlanning;

import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.graphSearch.listeners.PlannerGoalRecommendationListener;

public class PlannerGoalRecommendationHolder implements PlannerGoalRecommendationListener
{
   private final int stageId;
   private PlannerGoalRecommendationHandler goalHandler;

   public PlannerGoalRecommendationHolder(int stageId)
   {
      this.stageId = stageId;
   }

   public void setPlannerGoalRecommendationHandler(PlannerGoalRecommendationHandler goalHandler)
   {
      this.goalHandler = goalHandler;
   }

   @Override
   public void notifyWithPlannerGoalRecommendation(FootstepPlannerGoal footstepPlannerGoal)
   {
      goalHandler.notifyWithPlannerGoalRecommendation(footstepPlannerGoal, stageId);
   }
}
