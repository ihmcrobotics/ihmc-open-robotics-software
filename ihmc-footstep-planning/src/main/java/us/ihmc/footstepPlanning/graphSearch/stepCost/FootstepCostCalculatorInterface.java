package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public interface FootstepCostCalculatorInterface
{
   double computeCost(FootstepNode candidateStep, FootstepNode stanceStep, FootstepNode startOfSwing);
}
