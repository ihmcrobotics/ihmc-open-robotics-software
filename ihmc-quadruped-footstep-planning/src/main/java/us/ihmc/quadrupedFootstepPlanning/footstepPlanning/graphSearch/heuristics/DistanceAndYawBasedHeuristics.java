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

      double totalCost = parameters.getYawWeight() * Math.abs(yaw) + parameters.getCostPerStep() * minSteps;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         double euclideanDistance = Math.sqrt(MathTools.square(node.getX(robotQuadrant) - goalNode.getX(robotQuadrant)) + MathTools
               .square(node.getY(robotQuadrant) - goalNode.getY(robotQuadrant)));

         totalCost += distanceWeight * euclideanDistance;
      }

      return totalCost;
   }
}
