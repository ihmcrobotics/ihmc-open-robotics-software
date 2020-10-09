package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public interface FootstepNodeCheckerInterface
{
   boolean isNodeValid(FootstepNode candidateStep, FootstepNode stanceStep, FootstepNode startOfSwing);
}
