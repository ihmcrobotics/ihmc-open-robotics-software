package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class HeightCost implements FootstepCost
{
   private final BooleanSupplier useQuadraticHeightCost;
   private final LinearHeightCost linearHeightCost;
   private final QuadraticHeightCost quadraticHeightCost;

   public HeightCost(FootstepPlannerParametersReadOnly parameters, FootstepNodeSnapperReadOnly snapper)
   {
      this(parameters::useQuadraticHeightCost, parameters::getStepUpWeight, parameters::getStepDownWeight, snapper);
   }

   public HeightCost(BooleanSupplier useQuadraticHeightCost, DoubleSupplier stepUpWeight, DoubleSupplier stepDownWeight, FootstepNodeSnapperReadOnly snapper)
   {
      this.useQuadraticHeightCost = useQuadraticHeightCost;
      linearHeightCost = new LinearHeightCost(stepUpWeight, stepDownWeight, snapper);
      quadraticHeightCost = new QuadraticHeightCost(stepUpWeight, stepDownWeight, snapper);
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      if (useQuadraticHeightCost.getAsBoolean())
         return quadraticHeightCost.compute(startNode, endNode);
      else
         return linearHeightCost.compute(startNode, endNode);
   }
}
