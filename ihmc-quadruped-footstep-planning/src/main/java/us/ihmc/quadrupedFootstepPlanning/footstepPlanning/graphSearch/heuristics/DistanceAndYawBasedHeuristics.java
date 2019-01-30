package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.AngleTools;

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

      double bodyBasedHeuristicCost = distanceWeight * node.euclideanDistance(goalNode);
      double quadrantBasedHeuristicCost = distanceWeight * node.quadrantEuclideanDistance(node.getMovingQuadrant(), goalNode);
//      double yawHeuristicCost = parameters.getYawWeight() * Math.abs(yaw);
      double yawHeuristicCost = 0.0;
      double stepHeuristicCost = parameters.getCostPerStep() * minSteps;

//      return 0.5 * (bodyBasedHeuristicCost + quadrantBasedHeuristicCost) + yawHeuristicCost + stepHeuristicCost;
      return  quadrantBasedHeuristicCost + yawHeuristicCost + stepHeuristicCost;
//      return  bodyBasedHeuristicCost + yawHeuristicCost + stepHeuristicCost;
   }
}
