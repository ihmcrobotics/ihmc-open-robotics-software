package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import java.util.HashSet;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;

/**
 * This class expands nodes in a 8 connected way.
 * It does not account for turn footsteps. Yaw angle of potential footstep neighbour by default would be zero.
 * Very small 5cm steps.
 */
public class SimpleGridResolutionBasedExpansion implements FootstepNodeExpansion
{
   @Override
   public HashSet<FootstepNode> expandNode(FootstepNode node)
   {
      HashSet<FootstepNode> neighbors = new HashSet<>();
      for (int i = -1; i <= 1; i++)
      {
         for (int j = -1; j <= 1; j++)
         {
            if (i == 0 && j == 0)
               continue;
            double xOffset = LatticeNode.gridSizeXY * i;
            double yOffset = LatticeNode.gridSizeXY * j;
            neighbors.add(new FootstepNode(node.getX() + xOffset, node.getY() + yOffset));
         }
      }
      return neighbors;
   }
}
