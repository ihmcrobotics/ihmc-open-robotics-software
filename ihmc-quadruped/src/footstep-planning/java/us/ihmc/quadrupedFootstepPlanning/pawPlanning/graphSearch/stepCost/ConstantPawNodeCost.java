package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;

public class ConstantPawNodeCost implements PawNodeCost
{
   private final double cost;

   public ConstantPawNodeCost(double cost)
   {
      this.cost = cost;
   }

   @Override
   public double compute(PawNode startNode, PawNode endNode)
   {
      return cost;
   }
}
