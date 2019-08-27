package us.ihmc.footstepPlanning.graphSearch.heuristics;

import java.util.Comparator;

import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.robotSide.SideDependentList;

public class NodeComparator implements Comparator<FootstepNode>
{
   private final FootstepGraph graph;
   private final CostToGoHeuristics heuristics;

   public NodeComparator(FootstepGraph graph, CostToGoHeuristics heuristics)
   {
      this.graph = graph;
      this.heuristics = heuristics;
   }

   @Override
   public int compare(FootstepNode o1, FootstepNode o2)
   {
      double cost1 = graph.getCostFromStart(o1) + heuristics.compute(o1);
      double cost2 = graph.getCostFromStart(o2) + heuristics.compute(o2);
      if (cost1 == cost2) return 0;
      return cost1 < cost2 ? -1 : 1;
   }
}
