package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;

public interface CustomFootstepChecker
{
   boolean isStepValid(DiscreteFootstep candidateFootstep, DiscreteFootstep stanceNode);

   BipedalFootstepPlannerNodeRejectionReason getRejectionReason();
}
