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
      if (bodyDistance < 1.0)
      {
         double angleDifference = AngleTools.computeAngleDifferenceMinusPiToPi(node.getNominalYaw(), goalNode.getNominalYaw());
         yawHeuristicCost = parameters.getYawWeight() * Math.abs(angleDifference);
      }
      else
      {
         Point2DReadOnly goalCenter = goalNode.getOrComputeXGaitCenterPoint();
         Point2DReadOnly nodeCenter = node.getOrComputeXGaitCenterPoint();
         Vector2D headingVector = new Vector2D();
         headingVector.sub(goalCenter, nodeCenter);

         double headingAngle = headingVector.angle(forward);
         double angleDifference = AngleTools.computeAngleDifferenceMinusPiToPi(node.getNominalYaw(), headingAngle);
         yawHeuristicCost = parameters.getYawWeight() * Math.abs(angleDifference);

      }

      double stepHeuristicCost = parameters.getCostPerStep() * minSteps;

      return yawHeuristicCost + stepHeuristicCost + parameters.getDistanceHeuristicWeight() * bodyDistance;
   }
}
