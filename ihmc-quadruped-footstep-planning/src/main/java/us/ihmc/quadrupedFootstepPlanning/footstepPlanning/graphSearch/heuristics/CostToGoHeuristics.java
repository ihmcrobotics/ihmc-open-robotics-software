package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.yoVariables.providers.DoubleProvider;

public abstract class CostToGoHeuristics
{
   private final DoubleProvider weight;

   public CostToGoHeuristics(DoubleProvider weight)
   {
      this.weight = weight;
   }

   public double getWeight()
   {
      return weight.getValue();
   }

   public double compute(FootstepNode node, FootstepNode goalNode)
   {
      return weight.getValue() * computeHeuristics(node, goalNode);
   }

   protected abstract double computeHeuristics(FootstepNode node, FootstepNode goalNode);
}
