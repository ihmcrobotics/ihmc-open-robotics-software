package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;

public abstract class CostToGoHeuristics
{
   protected final FootstepPlannerParameters parameters;

   public CostToGoHeuristics(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   public double getWeight()
   {
      return parameters.getHeuristicsInflationWeight();
   }

   public double compute(FootstepNode node, FootstepNode goalNode)
   {
      return parameters.getHeuristicsInflationWeight() * computeHeuristics(node, goalNode);
   }

   protected abstract double computeHeuristics(FootstepNode node, FootstepNode goalNode);
}
