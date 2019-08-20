package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;

import java.util.ArrayList;
import java.util.List;

public class CompositePawPlanningCostToGoHeuristics extends PawPlanningCostToGoHeuristics
{
   private final List<PawPlanningCostToGoHeuristics> costToGoHeuristics;

   public CompositePawPlanningCostToGoHeuristics(PawPlannerParameters parameters)
   {
      super(parameters);

      this.costToGoHeuristics = new ArrayList<>();
   }

   public CompositePawPlanningCostToGoHeuristics(PawPlannerParameters parameters, List<PawPlanningCostToGoHeuristics> costToGoHeuristics)
   {
      super(parameters);

      this.costToGoHeuristics = costToGoHeuristics;
   }

   public void addCostToGoHeuristic(PawPlanningCostToGoHeuristics costToGoHeuristic)
   {
      costToGoHeuristics.add(costToGoHeuristic);
   }

   @Override
   protected double computeHeuristics(PawNode node, PawNode goalNode)
   {
      double cost = 0.0;
      for (PawPlanningCostToGoHeuristics pawCost : costToGoHeuristics)
         cost += pawCost.computeHeuristics(node, goalNode);

      return cost;
   }
}
