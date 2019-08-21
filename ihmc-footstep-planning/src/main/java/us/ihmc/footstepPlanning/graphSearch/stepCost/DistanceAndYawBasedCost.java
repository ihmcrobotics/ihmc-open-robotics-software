package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;

public class DistanceAndYawBasedCost implements FootstepCost
{
   private final FootstepPlannerParametersReadOnly parameters;

   private final EuclideanDistanceAndYawBasedCost euclideanCost;
   private final QuadraticDistanceAndYawCost quadraticCost;

   public DistanceAndYawBasedCost(FootstepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;

      euclideanCost = new EuclideanDistanceAndYawBasedCost(parameters);
      quadraticCost = new QuadraticDistanceAndYawCost(parameters);
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      if (parameters.useQuadraticDistanceCost())
         return quadraticCost.compute(startNode, endNode);
      else
         return euclideanCost.compute(startNode, endNode);
   }
}
