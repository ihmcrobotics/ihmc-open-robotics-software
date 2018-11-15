package us.ihmc.avatar.footstepPlanning;

import us.ihmc.footstepPlanning.FootstepPlanningResult;

public interface PlannerCompletionCallback
{
   void pathPlanningIsComplete(FootstepPlanningResult pathPlanningResult, PathPlanningStage stageFinished);
   void stepPlanningIsComplete(FootstepPlanningResult stepPlanningResult, FootstepPlanningStage stageFinished);
}
