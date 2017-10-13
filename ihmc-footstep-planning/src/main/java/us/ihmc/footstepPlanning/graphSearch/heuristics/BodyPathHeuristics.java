package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.stepCost.DistanceAndYawBasedCost;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class BodyPathHeuristics extends CostToGoHeuristics
{
   private static final double pathViolationWeight = 30.0;
   private final BodyPathPlanner bodyPath;
   private final FootstepPlannerParameters parameters;

   public BodyPathHeuristics(YoVariableRegistry registry, FootstepPlannerParameters parameters, BodyPathPlanner bodyPath)
   {
      super(registry);
      this.bodyPath = bodyPath;
      this.parameters = parameters;
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      Point2D midFootPoint = DistanceAndYawBasedCost.computeMidFootPoint(node, parameters.getIdealFootstepWidth());
      Pose2D closestPointOnPath = new Pose2D();
      double alpha = bodyPath.getClosestPoint(midFootPoint, closestPointOnPath);
      double distanceToPath = closestPointOnPath.getPosition().distance(midFootPoint);
      double pathLength = bodyPath.computePathLength(alpha);
      double remainingDistance = pathLength + pathViolationWeight * distanceToPath;

      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(node.getYaw(), closestPointOnPath.getYaw());
      double minSteps = Math.floor(remainingDistance / parameters.getMaximumStepReach());
      return remainingDistance + parameters.getYawWeight() * Math.abs(yaw) + parameters.getCostPerStep() * minSteps;
   }
}
