package us.ihmc.footstepPlanning.aStar;

public abstract class CostToGoHeuristics
{
   private double weight = 1.0;

   public void setWeight(double weight)
   {
      this.weight = Math.max(0.0, weight);
   }

   public double getWeight()
   {
      return weight;
   }

   public double compute(FootstepNode node, FootstepNode goalNode)
   {
      return weight * computeHeuristics(node, goalNode);
   }

   protected abstract double computeHeuristics(FootstepNode node, FootstepNode goalNode);
}
