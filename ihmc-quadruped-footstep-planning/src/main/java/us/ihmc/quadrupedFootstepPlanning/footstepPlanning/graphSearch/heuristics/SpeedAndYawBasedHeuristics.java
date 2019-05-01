package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class SpeedAndYawBasedHeuristics extends CostToGoHeuristics
{
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final FootstepNodeSnapper snapper;

   public SpeedAndYawBasedHeuristics(FootstepNodeSnapper snapper, FootstepPlannerParameters parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      super(parameters);

      this.snapper = snapper;
      this.xGaitSettings = xGaitSettings;
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      double bodyDistance = node.euclideanDistance(goalNode);
      double desiredSpeed = parameters.getMaxWalkingSpeedMultiplier() * xGaitSettings.getMaxSpeed();
      double minSteps = bodyDistance / desiredSpeed;

      double referenceYaw = computeReferenceYaw(node, goalNode);
      double angleDifference = AngleTools.computeAngleDifferenceMinusPiToPi(node.getNominalYaw(), referenceYaw);
      double yawHeuristicCost = parameters.getYawWeight() * Math.abs(angleDifference);

      double stepHeuristicCost = 4.0 * parameters.getCostPerStep() * minSteps;

      RigidBodyTransform startNodeTransform = new RigidBodyTransform();
      RigidBodyTransform endNodeTransform = new RigidBodyTransform();

      double heightCost = 0.0;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int endNodeXIndex = goalNode.getXIndex(robotQuadrant);
         int endNodeYIndex = goalNode.getYIndex(robotQuadrant);
         int startNodeXIndex = node.getXIndex(robotQuadrant);
         int startNodeYIndex = node.getYIndex(robotQuadrant);

         FootstepNodeSnapData endNodeData = snapper.getSnapData(endNodeXIndex, endNodeYIndex);
         FootstepNodeSnapData startNodeData = snapper.getSnapData(startNodeXIndex, startNodeYIndex);

         if (startNodeData == null || endNodeData == null)
            return 0.0;

         FootstepNodeTools.getSnappedNodeTransformToWorld(startNodeXIndex, startNodeYIndex, startNodeData.getSnapTransform(), startNodeTransform);
         FootstepNodeTools.getSnappedNodeTransformToWorld(endNodeXIndex, endNodeYIndex, endNodeData.getSnapTransform(), endNodeTransform);

         double heightChange = endNodeTransform.getTranslationVector().getZ() - startNodeTransform.getTranslationVector().getZ();

         if (heightChange > 0.0)
            heightCost += parameters.getStepUpWeight() * heightChange;
         else
            heightCost += -parameters.getStepDownWeight() * heightChange;
      }

      return yawHeuristicCost + stepHeuristicCost + heightCost + parameters.getDistanceHeuristicWeight() * bodyDistance;
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
