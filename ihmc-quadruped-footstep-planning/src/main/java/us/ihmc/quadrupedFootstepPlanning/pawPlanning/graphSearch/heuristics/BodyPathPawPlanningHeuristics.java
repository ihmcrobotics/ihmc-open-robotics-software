package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanHolder;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNodeTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class BodyPathPawPlanningHeuristics extends PawPlanningCostToGoHeuristics
{
   private static final double pathViolationWeight = 10.0;
   private static final double distanceFromPathTolerance = 0.2;
   private static final double deltaYawFromReferenceTolerance = 0.2;
   private static final double finalTurnProximity = 0.25;

   private final BodyPathPlanHolder bodyPath;
   private final PawNodeSnapper snapper;

   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;

   private double goalAlpha = 1.0;

   public BodyPathPawPlanningHeuristics(PawStepPlannerParametersReadOnly parameters, BodyPathPlanHolder bodyPath,
                                        QuadrupedXGaitSettingsReadOnly xGaitSettings, PawNodeSnapper snapper)
   {
      super(parameters);

      this.bodyPath = bodyPath;
      this.xGaitSettings = xGaitSettings;
      this.snapper = snapper;
   }

   @Override
   protected double computeHeuristics(PawNode node, PawNode goalNode)
   {
      Point2DReadOnly xGaitCenterPoint = node.getOrComputeXGaitCenterPoint();
      Pose2D closestPointOnPath = new Pose2D();

      double alpha = bodyPath.getClosestPoint(xGaitCenterPoint, closestPointOnPath);
      alpha = MathTools.clamp(alpha, 0.0, goalAlpha);
      bodyPath.getPointAlongPath(alpha, closestPointOnPath);

      double distanceToPath = closestPointOnPath.getPosition().distance(xGaitCenterPoint);
      double croppedDistanceToPath = Math.max(0.0, distanceToPath - distanceFromPathTolerance);

      double remainingPathLength = bodyPath.computePathLength(alpha) - bodyPath.computePathLength(goalAlpha);
      double remainingDistance = remainingPathLength + distanceToPath - croppedDistanceToPath;
      double pathDistanceViolationCost = pathViolationWeight * croppedDistanceToPath;


      double referenceYaw = computeReferenceGoalYaw(node, goalNode, closestPointOnPath.getYaw());

      double yawDifferenceFromReference = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(node.getStepYaw(), referenceYaw));
      double remainingYawToGoal = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(goalNode.getStepYaw(), referenceYaw));

      double croppedYawDifferenceFromReference = Math.max(0.0, yawDifferenceFromReference - deltaYawFromReferenceTolerance);

      double remainingYaw = remainingYawToGoal + yawDifferenceFromReference - croppedYawDifferenceFromReference;
      double pathYawViolationCost = pathViolationWeight * croppedYawDifferenceFromReference;

      double desiredSpeed = parameters.getMaxWalkingSpeedMultiplier() * xGaitSettings.getMaxSpeed();
      double minSteps = 4.0 * remainingDistance / desiredSpeed;

      double heightCost = 0.0;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int goalNodeXIndex = goalNode.getXIndex(robotQuadrant);
         int goalNodeYIndex = goalNode.getYIndex(robotQuadrant);
         int nodeXIndex = node.getXIndex(robotQuadrant);
         int nodeYIndex = node.getYIndex(robotQuadrant);

         PawNodeSnapData goalNodeData = snapper.snapPawNode(robotQuadrant, goalNodeXIndex, goalNodeYIndex);
         PawNodeSnapData nodeData = snapper.snapPawNode(robotQuadrant, nodeXIndex, nodeYIndex);

         if (nodeData == null || goalNodeData == null)
         {
            heightCost = 0.0;
            break;
         }

         RigidBodyTransform nodeTransform = new RigidBodyTransform();
         RigidBodyTransform goalNodeTransform = new RigidBodyTransform();

         PawNodeTools.getSnappedNodeTransformToWorld(nodeXIndex, nodeYIndex, nodeData.getSnapTransform(), nodeTransform);
         PawNodeTools.getSnappedNodeTransformToWorld(goalNodeXIndex, goalNodeYIndex, goalNodeData.getSnapTransform(), goalNodeTransform);

         if (!nodeTransform.containsNaN() && !goalNodeTransform.containsNaN())
         {
            double heightChange = goalNodeTransform.getTranslationVector().getZ() - nodeTransform.getTranslationVector().getZ();

            if (heightChange > 0.0)
               heightCost += parameters.getStepUpWeight() * heightChange;
            else
               heightCost += Math.abs(parameters.getStepDownWeight() * heightChange);
         }
      }

      double distanceCost = parameters.getDistanceWeight() * remainingDistance;
      double yawCost = parameters.getYawWeight() * remainingYaw;
      double stepCost = parameters.getCostPerStep() * minSteps;

      return distanceCost + yawCost + stepCost + + pathDistanceViolationCost + pathYawViolationCost + heightCost;
   }

   public void setGoalAlpha(double alpha)
   {
      goalAlpha = alpha;
   }

   private double computeReferenceGoalYaw(PawNode node, PawNode goalNode, double pathHeading)
   {
      double distanceToGoal = node.euclideanDistance(goalNode);
      double finalTurnProximity = this.finalTurnProximity;//parameters.getFinalTurnProximity();

      double minimumBlendDistance = 0.75 * finalTurnProximity;
      double maximumBlendDistance = 1.25 * finalTurnProximity;

      double yawMultiplier;
      if(distanceToGoal < minimumBlendDistance)
         yawMultiplier = 0.0;
      else if(distanceToGoal > maximumBlendDistance)
         yawMultiplier = 1.0;
      else
         yawMultiplier = (distanceToGoal - minimumBlendDistance) / (maximumBlendDistance - minimumBlendDistance);

      return AngleTools.interpolateAngle(goalNode.getStepYaw(), pathHeading, yawMultiplier);
   }
}
