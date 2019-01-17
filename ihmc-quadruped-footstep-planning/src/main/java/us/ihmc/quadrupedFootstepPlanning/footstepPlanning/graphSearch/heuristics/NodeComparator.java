package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;

import java.util.Comparator;

public class NodeComparator implements Comparator<FootstepNode>
{
   private final FootstepGraph graph;
   private final FootstepNode goalNode;
   private final CostToGoHeuristics heuristics;

   public NodeComparator(FootstepGraph graph, FootstepNode goalNode, CostToGoHeuristics heuristics)
   {
      this.graph = graph;
      this.goalNode = goalNode;
      this.heuristics = heuristics;
   }

   @Override
   public int compare(FootstepNode o1, FootstepNode o2)
   {
      double cost1 = graph.getCostFromStart(o1) + heuristics.compute(o1, goalNode);
      double cost2 = graph.getCostFromStart(o2) + heuristics.compute(o2, goalNode);
      if (cost1 == cost2) return 0;
      return cost1 < cost2 ? -1 : 1;
   }
}
