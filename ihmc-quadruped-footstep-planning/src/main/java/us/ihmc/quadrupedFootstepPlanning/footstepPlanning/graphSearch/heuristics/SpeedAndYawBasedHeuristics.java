package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.geometry.AngleTools;

public class SpeedAndYawBasedHeuristics extends CostToGoHeuristics
{
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final static Vector2D forward = new Vector2D(1.0, 0.0);

   public SpeedAndYawBasedHeuristics(FootstepPlannerParameters parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      super(parameters);

      this.xGaitSettings = xGaitSettings;
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      double bodyDistance = node.euclideanDistance(goalNode);
      double desiredSpeed = parameters.getMaxWalkingSpeedMultiplier() * xGaitSettings.getMaxSpeed();
      double minSteps = bodyDistance / desiredSpeed;

      double yawHeuristicCost;
      double referenceYaw = computeReferenceYaw(node, goalNode);


      double angleDifference = AngleTools.computeAngleDifferenceMinusPiToPi(node.getNominalYaw(), referenceYaw);
      yawHeuristicCost = parameters.getYawWeight() * Math.abs(angleDifference);

      double stepHeuristicCost = 4.0 * parameters.getCostPerStep() * minSteps;

      return yawHeuristicCost + stepHeuristicCost + parameters.getDistanceHeuristicWeight() * bodyDistance;
   }

   private double computeReferenceYaw(FootstepNode node, FootstepNode goalNode)
   {
      double distanceToGoal = node.euclideanDistance(goalNode);
      double finalTurnProximity = 1.0;

      double minimumBlendDistance = 0.75 * finalTurnProximity;
      double maximumBlendDistance = 1.25 * finalTurnProximity;

      double pathHeading = Math.atan2(goalNode.getOrComputeXGaitCenterPoint().getY() - node.getOrComputeXGaitCenterPoint().getY(),
                                      goalNode.getOrComputeXGaitCenterPoint().getX() - node.getOrComputeXGaitCenterPoint().getX());
      pathHeading = AngleTools.trimAngleMinusPiToPi(pathHeading);

      double yawMultiplier;
      if (distanceToGoal < minimumBlendDistance)
         yawMultiplier = 0.0;
      else if(distanceToGoal > maximumBlendDistance)
         yawMultiplier = 1.0;
      else
         yawMultiplier = (distanceToGoal - minimumBlendDistance) / (maximumBlendDistance - minimumBlendDistance);

      double referenceHeading = yawMultiplier * pathHeading;
      referenceHeading += (1.0 - yawMultiplier) * goalNode.getNominalYaw();
      return AngleTools.trimAngleMinusPiToPi(referenceHeading);
   }
}
