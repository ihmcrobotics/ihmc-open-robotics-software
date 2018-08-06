package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public class QuadraticHeightCost implements FootstepCost
{
   private static final double stepUpCost = 1.0;
   private static final double stepHeightScalar = 10.0;
   private static final double stepDownCost = 2.0;

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      if (!startNode.hasZ() || !endNode.hasZ())
         return 0.0;

      double heightChange = endNode.getZ() - startNode.getZ();

      if (heightChange > 0.0)
         return stepUpCost * Math.pow(stepHeightScalar * heightChange, 2.0);
      else
         return stepDownCost * Math.pow(stepHeightScalar * heightChange, 2.0);
   }
}
