package us.ihmc.pathPlanning.graph.structure;

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
