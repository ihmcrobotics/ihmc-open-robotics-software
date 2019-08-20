package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class BodyPathHeuristics extends CostToGoHeuristics
{
   private final BodyPathPlanner bodyPath;
   private final FootstepPlannerParametersReadOnly parameters;

   private double goalAlpha = 1.0;

   public BodyPathHeuristics(DoubleProvider weight, FootstepPlannerParametersReadOnly parameters, BodyPathPlanner bodyPath)
   {
      super(weight);

      this.bodyPath = bodyPath;
      this.parameters = parameters;
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      Point2D midFootPoint = node.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
      Pose2D closestPointOnPath = new Pose2D();

      double alpha = bodyPath.getClosestPoint(midFootPoint, closestPointOnPath);
      alpha = MathTools.clamp(alpha, 0.0, goalAlpha);
      bodyPath.getPointAlongPath(alpha, closestPointOnPath);

      double distanceToPath = closestPointOnPath.getPosition().distance(midFootPoint);
      double croppedDistanceToPath = Math.max(0.0, distanceToPath - parameters.getDistanceFromPathTolerance());

      double remainingPathLength = bodyPath.computePathLength(alpha) - bodyPath.computePathLength(goalAlpha);
      double remainingDistance = remainingPathLength + distanceToPath - croppedDistanceToPath;
      double pathDistanceViolationCost = parameters.getBodyPathViolationWeight() * croppedDistanceToPath;

      double referenceYaw = computeReferenceGoalYaw(node, goalNode, closestPointOnPath.getYaw());

      double yawDifferenceFromReference = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(node.getYaw(), referenceYaw));
      double remainingYawToGoal = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(goalNode.getYaw(), referenceYaw));

      double croppedYawDifferenceFromReference = Math.max(0.0, yawDifferenceFromReference - parameters.getDeltaYawFromReferenceTolerance());

      double remainingYaw = remainingYawToGoal + yawDifferenceFromReference - croppedYawDifferenceFromReference;
      double pathYawViolationCost = parameters.getBodyPathViolationWeight() * croppedYawDifferenceFromReference;

      double minSteps = remainingDistance / parameters.getMaximumStepReach() + Math.abs(remainingYaw) / (0.5 * parameters.getMaximumStepYaw());
      return remainingDistance + pathDistanceViolationCost + pathYawViolationCost + parameters.getYawWeight() * remainingYaw + parameters.getCostPerStep() * minSteps;
   }


   private double computeReferenceGoalYaw(FootstepNode node, FootstepNode goalNode, double pathHeading)
   {
      double distanceToGoal = node.euclideanDistance(goalNode);
      double finalTurnProximity = parameters.getFinalTurnBodyPathProximity();

      double minimumBlendDistance = (1.0 - parameters.getFinalTurnProximityBlendFactor()) * finalTurnProximity;
      double maximumBlendDistance = (1.0 + parameters.getFinalTurnProximityBlendFactor()) * finalTurnProximity;

      double yawMultiplier;
      if(distanceToGoal < minimumBlendDistance)
         yawMultiplier = 0.0;
      else if(distanceToGoal > maximumBlendDistance)
         yawMultiplier = 1.0;
      else
         yawMultiplier = (distanceToGoal - minimumBlendDistance) / (maximumBlendDistance - minimumBlendDistance);

      return AngleTools.interpolateAngle(goalNode.getYaw(), pathHeading, yawMultiplier);
   }

   public void setGoalAlpha(double alpha)
   {
      goalAlpha = alpha;
   }
}
