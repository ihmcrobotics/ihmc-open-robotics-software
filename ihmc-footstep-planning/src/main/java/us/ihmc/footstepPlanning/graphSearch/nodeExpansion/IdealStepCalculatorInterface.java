package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public interface IdealStepCalculatorInterface
{
   FootstepNode computeIdealStep(FootstepNode stanceNode, FootstepNode startOfSwing);
}
