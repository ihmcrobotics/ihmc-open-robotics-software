package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class CostToGoHeuristics
{
   private DoubleProvider heuristicsInflationWeight;
   protected final FootstepPlannerParameters parameters;

   public CostToGoHeuristics(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   public void setHeuristicsInflationWeight(DoubleProvider heuristicsInflationWeight)
   {
      this.heuristicsInflationWeight = heuristicsInflationWeight;
   }

   public double compute(FootstepNode node, FootstepNode goalNode)
   {
      return heuristicsInflationWeight.getValue() * computeHeuristics(node, goalNode);
   }

   public double getWeight()
   {
      return heuristicsInflationWeight.getValue();
   }

   protected abstract double computeHeuristics(FootstepNode node, FootstepNode goalNode);
}
