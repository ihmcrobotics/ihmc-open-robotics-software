package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;

import java.util.function.DoubleSupplier;

public class PerStepCost implements FootstepCost
{
   private final DoubleSupplier costPerStep;

   public PerStepCost(DoubleSupplier costPerStep)
   {
      this.costPerStep = costPerStep;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      return costPerStep.getAsDouble();
   }
}
