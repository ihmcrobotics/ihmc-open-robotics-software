package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.footstepPlanning.aStar.CostToGoHeuristics;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
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
