package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParametersReadOnly;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.geometry.AngleTools;

public class PawSpeedBasedHeuristics extends PawPlanningCostToGoHeuristics
{
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;

   public PawSpeedBasedHeuristics(PawPlannerParametersReadOnly parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      super(parameters);

      this.xGaitSettings = xGaitSettings;
   }

   @Override
   protected double computeHeuristics(PawNode node, PawNode goalNode)
   {
      double bodyDistance = node.euclideanDistance(goalNode);
      double yawDistance = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(node.getStepYaw(), goalNode.getStepYaw()));
      double desiredSpeed = parameters.getMaxWalkingSpeedMultiplier() * xGaitSettings.getMaxSpeed();
      double desiredYawSpeed = xGaitSettings.getMaxYawSpeedFraction() * desiredSpeed;
      double maxYaw = Math.min(desiredYawSpeed * (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration()),
                               0.5 * (parameters.getMaximumStepYawOutward() - parameters.getMaximumStepYawInward()));
      double minDistanceSteps = 4.0 * bodyDistance / desiredSpeed;
      double minYawSteps = 2.0 * yawDistance / maxYaw;

      return parameters.getCostPerStep() * Math.max(minDistanceSteps, minYawSteps);
   }
}
