package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;

public class DistanceAndYawBasedCost implements FootstepCost
{
   private final EuclideanDistanceAndYawBasedCost euclideanCost;

   public DistanceAndYawBasedCost(FootstepPlannerParameters parameters)
   {
      euclideanCost = new EuclideanDistanceAndYawBasedCost(parameters);
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      return euclideanCost.compute(startNode, endNode);
   }
}
