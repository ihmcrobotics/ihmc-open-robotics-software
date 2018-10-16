package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;

public class DistanceAndYawBasedCost implements FootstepCost
{
   private final FootstepPlannerParameters parameters;
   private final FootstepPlannerCostParameters costParameters;

   public DistanceAndYawBasedCost(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
      costParameters = parameters.getCostParameters();
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      Point2D startPoint = startNode.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
      Point2D endPoint = endNode.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
      double euclideanDistance = startPoint.distance(endPoint);
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(startNode.getYaw(), endNode.getYaw());

      double distanceCost = 0.5 * (costParameters.getForwardWeight() + costParameters.getLateralWeight());

      return distanceCost * euclideanDistance + costParameters.getYawWeight() * Math.abs(yaw) + costParameters.getCostPerStep();
   }
}
