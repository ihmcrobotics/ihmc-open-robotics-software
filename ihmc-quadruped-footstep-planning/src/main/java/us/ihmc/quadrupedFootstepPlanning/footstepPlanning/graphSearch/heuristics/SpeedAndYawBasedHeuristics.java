package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.geometry.AngleTools;

public class SpeedAndYawBasedHeuristics extends CostToGoHeuristics
{
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;

   public SpeedAndYawBasedHeuristics(FootstepPlannerParameters parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      super(parameters);

      this.xGaitSettings = xGaitSettings;
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(node.getNominalYaw(), goalNode.getNominalYaw());
      double bodyDistance = node.euclideanDistance(goalNode);
      double minSteps = bodyDistance / parameters.getDesiredWalkingSpeed(xGaitSettings.getEndPhaseShift());

      double yawHeuristicCost = parameters.getYawWeight() * Math.abs(yaw);
      double stepHeuristicCost = parameters.getCostPerStep() * minSteps;

      return yawHeuristicCost + stepHeuristicCost;
   }
}
