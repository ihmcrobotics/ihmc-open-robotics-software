package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.footstepPlanning.aStar.FootstepCost;
import us.ihmc.footstepPlanning.aStar.FootstepNode;

public class EuclidianBasedCost implements FootstepCost
{
   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      return startNode.euclideanDistance(endNode);
   }
}
