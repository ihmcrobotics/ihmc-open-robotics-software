package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.footstepPlanning.aStar.CostToGoHeuristics;
import us.ihmc.footstepPlanning.aStar.FootstepNode;

public class ZeroHeuristics extends CostToGoHeuristics
{
   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      return 0.0;
   }
}
