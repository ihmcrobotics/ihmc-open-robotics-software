package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

public abstract class PawPlanningCostToGoHeuristics
{
   private DoubleProvider heuristicsInflationWeight;
   protected final PawStepPlannerParametersReadOnly parameters;

   public PawPlanningCostToGoHeuristics(PawStepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   public void setHeuristicsInflationWeight(DoubleProvider heuristicsInflationWeight)
   {
      this.heuristicsInflationWeight = heuristicsInflationWeight;
   }

   public double compute(PawNode node, PawNode goalNode)
   {
      return heuristicsInflationWeight.getValue() * computeHeuristics(node, goalNode);
   }

   public double getWeight()
   {
      return heuristicsInflationWeight.getValue();
   }

   protected abstract double computeHeuristics(PawNode node, PawNode goalNode);
}
