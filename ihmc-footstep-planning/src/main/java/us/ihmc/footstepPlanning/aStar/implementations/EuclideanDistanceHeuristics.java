package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.footstepPlanning.aStar.CostToGoHeuristics;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class EuclideanDistanceHeuristics extends CostToGoHeuristics
{
   public EuclideanDistanceHeuristics(YoVariableRegistry registry)
   {
      super(registry);
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      return node.euclideanDistance(goalNode);
   }
}
