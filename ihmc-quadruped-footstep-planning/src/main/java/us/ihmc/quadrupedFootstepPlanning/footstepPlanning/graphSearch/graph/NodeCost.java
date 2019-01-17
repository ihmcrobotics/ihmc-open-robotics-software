package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph;

public class NodeCost
{
   private final double cost;

   public NodeCost(double cost)
   {
      this.cost = cost;
   }

   public double getNodeCost()
   {
      return cost;
   }
}
