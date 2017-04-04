package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.footstepPlanning.aStar.FootstepCost;
import us.ihmc.footstepPlanning.aStar.FootstepNode;

public class EuclidianBasedCost implements FootstepCost
{
   private final double stepBaseCost;

   public EuclidianBasedCost()
   {
      this(0.0);
   }

   public EuclidianBasedCost(double stepBaseCost)
   {
      this.stepBaseCost = stepBaseCost;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      return startNode.euclideanDistance(endNode) + stepBaseCost;
   }
}
