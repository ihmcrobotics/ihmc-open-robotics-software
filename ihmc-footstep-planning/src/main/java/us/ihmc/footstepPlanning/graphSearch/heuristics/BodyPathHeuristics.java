package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class BodyPathHeuristics extends CostToGoHeuristics
{
   //TODO: Make these parameters or something, rather than hardcoded here.
   private static final double pathViolationWeight = 30.0;
   private static final double distanceFromPathTolerance = 0.2;
   private static final double deltaYawFromPathTolerance = 0.2;

   private final BodyPathPlanner bodyPath;
   private final FootstepPlannerParameters parameters;
   private final FootstepPlannerCostParameters costParameters;

   private double goalAlpha = 1.0;

   public BodyPathHeuristics(DoubleProvider weight, FootstepPlannerParameters parameters, BodyPathPlanner bodyPath)
   {
      super(weight);

      this.bodyPath = bodyPath;
      this.parameters = parameters;
      costParameters = parameters.getCostParameters();
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
      distanceToPath = Math.max(0.0, distanceToPath - distanceFromPathTolerance);

      double pathLength = bodyPath.computePathLength(alpha) - bodyPath.computePathLength(goalAlpha);
      double remainingDistance = pathLength + pathViolationWeight * distanceToPath;

      double yawDifferenceFromPathYaw = AngleTools.computeAngleDifferenceMinusPiToPi(node.getYaw(), closestPointOnPath.getYaw());
      yawDifferenceFromPathYaw = Math.max(0.0, distanceToPath - deltaYawFromPathTolerance);

      double yawCost = pathViolationWeight * yawDifferenceFromPathYaw;

      double minSteps = remainingDistance / parameters.getMaximumStepReach();
      return remainingDistance + costParameters.getYawWeight() * Math.abs(yawCost) + costParameters.getCostPerStep() * minSteps;
   }

   public void setGoalAlpha(double alpha)
   {
      goalAlpha = alpha;
   }
}
