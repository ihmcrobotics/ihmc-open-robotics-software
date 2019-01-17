package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class EuclideanDistanceHeuristics extends CostToGoHeuristics
{
   public EuclideanDistanceHeuristics(DoubleProvider weight)
   {
      super(weight);
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      return node.euclideanDistance(goalNode);
   }
}
