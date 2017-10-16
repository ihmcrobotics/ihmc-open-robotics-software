package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.robotics.geometry.AngleTools;

public class BodyPathCost implements FootstepCost
{
   private final DistanceAndYawBasedCost internalCost;
   private final BodyPathPlanner bodyPath;
   private final FootstepPlannerParameters parameters;

   public BodyPathCost(FootstepPlannerParameters parameters, BodyPathPlanner bodyPath)
   {
      internalCost = new DistanceAndYawBasedCost(parameters);
      this.bodyPath = bodyPath;
      this.parameters = parameters;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      Point2D endPoint = DistanceAndYawBasedCost.computeMidFootPoint(endNode, parameters.getIdealFootstepWidth());
      Pose2D closestPointOnPath = new Pose2D();
      bodyPath.getClosestPoint(endPoint, closestPointOnPath);
      double yawToPath = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(endNode.getYaw(), closestPointOnPath.getYaw()));
      double additionalPathViolationCost = parameters.getYawWeight() * yawToPath;

      return additionalPathViolationCost + internalCost.compute(startNode, endNode);
   }

}
