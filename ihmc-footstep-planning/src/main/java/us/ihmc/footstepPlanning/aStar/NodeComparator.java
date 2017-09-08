package us.ihmc.footstepPlanning.aStar;

import java.util.Comparator;

import us.ihmc.robotics.robotSide.SideDependentList;

public class NodeComparator implements Comparator<FootstepNode>
{
   private final FootstepGraph graph;
   private final SideDependentList<FootstepNode> goalNodes;
   private final CostToGoHeuristics heuristics;

   public NodeComparator(FootstepGraph graph, SideDependentList<FootstepNode> goalNodes, CostToGoHeuristics heuristics)
   {
      this.graph = graph;
      this.goalNodes = goalNodes;
      this.heuristics = heuristics;
   }

   @Override
   public int compare(FootstepNode o1, FootstepNode o2)
   {
      double cost1 = graph.getCostFromStart(o1) + heuristics.compute(o1, goalNodes.get(o1.getRobotSide()));
      double cost2 = graph.getCostFromStart(o2) + heuristics.compute(o2, goalNodes.get(o2.getRobotSide()));
      if (cost1 == cost2) return 0;
      return cost1 < cost2 ? -1 : 1;
   }
}
