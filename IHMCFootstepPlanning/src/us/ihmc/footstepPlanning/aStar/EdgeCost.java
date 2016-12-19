package us.ihmc.footstepPlanning.aStar;

public class EdgeCost
{
   private final double cost;

   public EdgeCost(double cost)
   {
      this.cost = cost;
   }

   public double getEdgeCost()
   {
      return cost;
   }
}
