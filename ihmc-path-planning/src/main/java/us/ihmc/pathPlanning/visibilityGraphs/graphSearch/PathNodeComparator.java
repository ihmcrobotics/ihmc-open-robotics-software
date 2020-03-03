package us.ihmc.pathPlanning.visibilityGraphs.graphSearch;

import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNode;

import java.util.Comparator;

public class PathNodeComparator implements Comparator<VisibilityGraphNode>
{
   private final EstimatedCostToGoal heuristics;

   public PathNodeComparator(EstimatedCostToGoal heuristics)
   {
      this.heuristics = heuristics;
   }

   @Override
   public int compare(VisibilityGraphNode nodeOne, VisibilityGraphNode nodeTwo)
   {
      if (nodeOne.equals(nodeTwo))
         return 0;

      double cost1 = nodeOne.getCostFromStart() + heuristics.compute(nodeOne);
      double cost2 = nodeTwo.getCostFromStart() + heuristics.compute(nodeTwo);
      if (cost1 == cost2)
         return 0;
      return cost1 < cost2 ? -1 : 1;
   }
}
