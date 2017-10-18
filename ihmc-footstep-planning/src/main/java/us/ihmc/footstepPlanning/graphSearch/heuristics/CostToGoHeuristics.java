package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class CostToGoHeuristics
{
   private final YoDouble weight;

   public CostToGoHeuristics(YoVariableRegistry registry)
   {
      weight = new YoDouble("HeuristicWeight", registry);
      weight.set(1.0);
   }

   public void setWeight(double weight)
   {
      this.weight.set(Math.max(0.0, weight));
   }

   public double getWeight()
   {
      return weight.getDoubleValue();
   }

   public double compute(FootstepNode node, FootstepNode goalNode)
   {
      return weight.getDoubleValue() * computeHeuristics(node, goalNode);
   }

   protected abstract double computeHeuristics(FootstepNode node, FootstepNode goalNode);
}
