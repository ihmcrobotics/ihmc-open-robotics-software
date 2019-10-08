package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawStepGraph;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;

import java.util.Comparator;

public class PawNodeComparator implements Comparator<PawNode>
{
   private final PawStepGraph graph;
   private final PawPlanningCostToGoHeuristics heuristics;

   public PawNodeComparator(PawStepGraph graph, PawPlanningCostToGoHeuristics heuristics)
   {
      this.graph = graph;
      this.heuristics = heuristics;
   }

   @Override
   public int compare(PawNode o1, PawNode o2)
   {
      double cost1 = graph.getCostFromStart(o1) + heuristics.compute(o1);
      double cost2 = graph.getCostFromStart(o2) + heuristics.compute(o2);
      if (cost1 == cost2)
         return 0;
      return cost1 < cost2 ? -1 : 1;
   }
}
