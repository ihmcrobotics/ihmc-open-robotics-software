package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph;

public class PawNodeCost
{
   private final double cost;

   public PawNodeCost(double cost)
   {
      this.cost = cost;
   }

   public double getNodeCost()
   {
      return cost;
   }
}
