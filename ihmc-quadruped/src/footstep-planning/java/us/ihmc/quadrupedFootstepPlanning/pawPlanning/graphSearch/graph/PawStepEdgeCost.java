package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph;

public class PawStepEdgeCost
{
   private final double cost;

   public PawStepEdgeCost(double cost)
   {
      this.cost = cost;
   }

   public double getEdgeCost()
   {
      return cost;
   }
}
