package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.commons.MathTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class DistanceAndYawBasedHeuristics extends CostToGoHeuristics
{
   public DistanceAndYawBasedHeuristics(FootstepPlannerParameters parameters)
   {
      super(parameters);
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(node.getNominalYaw(), goalNode.getNominalYaw());
      double bodyDistance = node.getOrComputeMidStancePoint().distance(node.getOrComputeMidStancePoint());
      double minSteps = 4.0 * bodyDistance / parameters.getMaximumStepCycleDistance();
      double distanceWeight = 0.5 * (parameters.getForwardWeight() + parameters.getLateralWeight());

      double bodyBasedHeuristicDistance = node.euclideanDistance(goalNode);
      double quadrantBasedDistance = node.quadrantEuclideanDistance(node.getMovingQuadrant(), goalNode);
      double averageDistance = 0.5 * (quadrantBasedDistance + bodyBasedHeuristicDistance);

      double yawHeuristicCost = parameters.getYawWeight() * Math.abs(yaw);
      double stepHeuristicCost = parameters.getCostPerStep() * minSteps;

      return distanceWeight * bodyBasedHeuristicDistance + yawHeuristicCost + stepHeuristicCost;
   }
}
