package us.ihmc.avatar.footstepPlanning;

import us.ihmc.footstepPlanning.FootstepPlanningResult;

public interface PlannerCompletionCallback
{
   void pathPlanningIsComplete(FootstepPlanningResult pathPlanningResult, FootstepPlanningStage stageFinished);
   void stepPlanningIsComplete(FootstepPlanningResult stepPlanningResult, FootstepPlanningStage stageFinished);
}
