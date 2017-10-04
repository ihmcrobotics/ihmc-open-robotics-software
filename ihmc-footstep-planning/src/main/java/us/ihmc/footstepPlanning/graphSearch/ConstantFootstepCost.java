package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.footstepPlanning.aStar.FootstepCost;
import us.ihmc.footstepPlanning.aStar.FootstepNode;

public class ConstantFootstepCost implements FootstepCost
{
   private final double cost;

   public ConstantFootstepCost(double cost)
   {
      this.cost = cost;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      return cost;
   }
}
