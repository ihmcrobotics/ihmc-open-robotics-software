package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;

import java.util.ArrayList;
import java.util.List;

public class CompositeCostToGoHeuristics extends CostToGoHeuristics
{
   private final List<CostToGoHeuristics> costToGoHeuristics;

   public CompositeCostToGoHeuristics(FootstepPlannerParameters parameters)
   {
      super(parameters);

      this.costToGoHeuristics = new ArrayList<>();
   }

   public CompositeCostToGoHeuristics(FootstepPlannerParameters parameters, List<CostToGoHeuristics> costToGoHeuristics)
   {
      super(parameters);

      this.costToGoHeuristics = costToGoHeuristics;
   }

   public void addCostToGoHeuristic(CostToGoHeuristics costToGoHeuristic)
   {
      costToGoHeuristics.add(costToGoHeuristic);
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      double cost = 0.0;
      for (CostToGoHeuristics footstepCost : costToGoHeuristics)
         cost += footstepCost.computeHeuristics(node, goalNode);

      return cost;
   }
}
