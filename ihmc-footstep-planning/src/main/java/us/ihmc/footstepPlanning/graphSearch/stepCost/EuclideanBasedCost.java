package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class EuclideanBasedCost implements FootstepCost
{
   private final static double defaultStepBaseCost = 0.0;
   private final YoDouble stepBaseCost;

   public EuclideanBasedCost(YoVariableRegistry registry)
   {
      this.stepBaseCost = new YoDouble("stepBaseCost", registry);
      stepBaseCost.set(defaultStepBaseCost);
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      return startNode.euclideanDistance(endNode) + stepBaseCost.getDoubleValue();
   }
}
