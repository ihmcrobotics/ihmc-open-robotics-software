package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.footstepPlanning.aStar.CostToGoHeuristics;
import us.ihmc.footstepPlanning.aStar.FootstepNode;

public class EuclidianDistanceHeuristics extends CostToGoHeuristics
{
   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      return node.euclideanDistance(goalNode);
   }
}
