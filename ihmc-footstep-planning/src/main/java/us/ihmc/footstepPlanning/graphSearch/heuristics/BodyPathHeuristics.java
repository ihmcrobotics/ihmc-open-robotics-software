package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanHolder;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class BodyPathHeuristics extends CostToGoHeuristics
{
   private final BodyPathPlanHolder bodyPath;
   private final FootstepPlannerParametersReadOnly parameters;
   private final Point2D midFootPoint = new Point2D();

   private double goalAlpha = 1.0;

   public BodyPathHeuristics(DoubleProvider weight, FootstepPlannerParametersReadOnly parameters, FootstepNodeSnapperReadOnly snapper, BodyPathPlanHolder bodyPath)
   {
      super(weight, parameters, snapper);

      this.bodyPath = bodyPath;
      this.parameters = parameters;
   }

   @Override
   protected double computeHeuristics(FramePose3DReadOnly pose)
   {
      Pose3D closestPointOnPath = new Pose3D();

      midFootPoint.set(pose.getPosition());
      double alpha = bodyPath.getClosestPoint(midFootPoint, closestPointOnPath);
      alpha = MathTools.clamp(alpha, 0.0, goalAlpha);
      bodyPath.getPointAlongPath(alpha, closestPointOnPath);

      double distanceToPath = closestPointOnPath.getPosition().distanceXY(midFootPoint);
      double croppedDistanceToPath = Math.max(0.0, distanceToPath - parameters.getDistanceFromPathTolerance());

      double remainingPathLength = bodyPath.computePathLength(alpha) - bodyPath.computePathLength(goalAlpha);
      double remainingDistance = remainingPathLength + distanceToPath - croppedDistanceToPath;
      double pathDistanceViolationCost = parameters.getBodyPathViolationWeight() * croppedDistanceToPath;

      double referenceYaw = computeReferenceGoalYaw(pose, closestPointOnPath.getYaw());

      double yawDifferenceFromReference = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(pose.getYaw(), referenceYaw));
      double remainingYawToGoal = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(goalPose.getYaw(), referenceYaw));

      double croppedYawDifferenceFromReference = Math.max(0.0, yawDifferenceFromReference - parameters.getDeltaYawFromReferenceTolerance());

      double remainingYaw = remainingYawToGoal + yawDifferenceFromReference - croppedYawDifferenceFromReference;
      double pathYawViolationCost = parameters.getBodyPathViolationWeight() * croppedYawDifferenceFromReference;

//      double minSteps = remainingDistance / parameters.getMaximumStepReach() + Math.abs(remainingYaw) / (0.5 * parameters.getMaximumStepYaw());
      int minSteps = Math.max(((int) (remainingDistance / parameters.getMaximumStepReach())), ((int) (Math.abs(remainingYaw) / (0.5 * parameters.getMaximumStepYaw()))));
      return remainingDistance + pathDistanceViolationCost + pathYawViolationCost + parameters.getYawWeight() * remainingYaw + parameters.getCostPerStep() * minSteps;
   }


   private double computeReferenceGoalYaw(FramePose3DReadOnly pose, double pathHeading)
   {
      double distanceToGoal = pose.getPosition().distanceXY(goalPose.getPosition());
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

      return AngleTools.interpolateAngle(goalPose.getYaw(), pathHeading, yawMultiplier);
   }

   public void setGoalAlpha(double alpha)
   {
      goalAlpha = alpha;
   }
}
