package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
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
