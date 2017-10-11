package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ZeroHeuristics extends CostToGoHeuristics
{
   public ZeroHeuristics(YoVariableRegistry registry)
   {
      super(registry);
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      return 0.0;
   }
}
