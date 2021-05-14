package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;

public interface FootstepCheckerInterface
{
   boolean isStepValid(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep, DiscreteFootstep startOfSwing);
}
