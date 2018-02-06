package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;

public class DistanceAndYawBasedCost implements FootstepCost
{
   private final FootstepPlannerParameters parameters;

   public DistanceAndYawBasedCost(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      Point2D startPoint = startNode.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
      Point2D endPoint = endNode.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
      double euclideanDistance = startPoint.distance(endPoint);
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(startNode.getYaw(), endNode.getYaw());
      return euclideanDistance + parameters.getYawWeight() * Math.abs(yaw) + parameters.getCostPerStep();
   }
}
