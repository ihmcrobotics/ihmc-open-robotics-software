package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.geometry.AngleTools;

public class SpeedBasedHeuristics extends CostToGoHeuristics
{
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;

   public SpeedBasedHeuristics(FootstepPlannerParameters parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      super(parameters);

      this.xGaitSettings = xGaitSettings;
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      double bodyDistance = node.euclideanDistance(goalNode);
      double yawDistance = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(node.getStepYaw(), goalNode.getStepYaw()));
      double desiredSpeed = parameters.getMaxWalkingSpeedMultiplier() * xGaitSettings.getMaxSpeed();
      double desiredYawSpeed = xGaitSettings.getMaxYawSpeedFraction() * desiredSpeed;
      double maxYaw = Math.min(desiredYawSpeed * (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration()),
                               0.5 * (parameters.getMaximumStepYaw() - parameters.getMinimumStepYaw()));
      double minDistanceSteps = 4.0 * bodyDistance / desiredSpeed;
      double minYawSteps = 2.0 * yawDistance / maxYaw;

      return parameters.getCostPerStep() * Math.max(minDistanceSteps, minYawSteps);
   }
}
