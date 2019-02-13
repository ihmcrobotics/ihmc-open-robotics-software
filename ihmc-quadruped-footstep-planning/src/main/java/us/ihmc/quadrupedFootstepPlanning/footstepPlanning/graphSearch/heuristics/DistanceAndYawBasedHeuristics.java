package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.AngleTools;

public class DistanceAndYawBasedHeuristics extends CostToGoHeuristics
{
   private boolean bodyHasReachedGoal = false;

   public DistanceAndYawBasedHeuristics(FootstepPlannerParameters parameters)
   {
      super(parameters);
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(node.getNominalYaw(), goalNode.getNominalYaw());
      double bodyDistance = node.getOrComputeXGaitCenterPoint().distance(node.getOrComputeXGaitCenterPoint());
      double minSteps = 2.0 * bodyDistance / parameters.getMaximumStepReach();
      double distanceWeight = 0.5 * (parameters.getForwardWeight() + parameters.getLateralWeight());

      double bodyBasedHeuristicDistance = node.euclideanDistance(goalNode);
      double quadrantBasedDistance = node.quadrantEuclideanDistance(node.getMovingQuadrant(), goalNode);
      double distance;
      if (bodyHasReachedGoal)
         distance = quadrantBasedDistance;
      else
         distance = bodyBasedHeuristicDistance;

      double yawHeuristicCost = parameters.getYawWeight() * Math.abs(yaw);
      double stepHeuristicCost = parameters.getCostPerStep() * minSteps;
      double distanceCost = distanceWeight * distance;

      return distanceCost + yawHeuristicCost + stepHeuristicCost;
   }
}
