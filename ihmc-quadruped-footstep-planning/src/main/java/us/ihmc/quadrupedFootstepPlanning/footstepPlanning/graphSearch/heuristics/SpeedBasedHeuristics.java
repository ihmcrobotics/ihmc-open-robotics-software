package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;

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
      double desiredSpeed = parameters.getMaxWalkingSpeedMultiplier() * xGaitSettings.getMaxSpeed();
      double minSteps = 4.0 * bodyDistance / desiredSpeed;

      return parameters.getCostPerStep() * minSteps;
   }
}
