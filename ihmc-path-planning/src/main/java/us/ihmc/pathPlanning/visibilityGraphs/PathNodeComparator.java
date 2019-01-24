package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNode;

import java.util.Comparator;

public class PathNodeComparator implements Comparator<VisibilityGraphNode>
{
   @Override
   public int compare(VisibilityGraphNode nodeOne, VisibilityGraphNode nodeTwo)
   {
      //TODO: Check the statement below. It might be false, since just doing compare not equals?
      //Note: Can only return 0 if the two nodes are ==.
      // This is because queue.remove(node) will remove the first one with .equals()
      if (nodeOne == nodeTwo)
         return 0;

      double cost1 = nodeOne.getCostFromStart() + nodeOne.getEstimatedCostToGoal();
      double cost2 = nodeTwo.getCostFromStart() + nodeTwo.getEstimatedCostToGoal();
      if (cost1 == cost2) return 0;
      return cost1 < cost2 ? -1 : 1;
   }
}
