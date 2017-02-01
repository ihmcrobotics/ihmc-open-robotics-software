package us.ihmc.footstepPlanning.aStar.implementations;

import javax.vecmath.Point2d;

import us.ihmc.footstepPlanning.aStar.CostToGoHeuristics;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;

public class DistanceAndYawBasedHeuristics extends CostToGoHeuristics
{
   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      Point2d goalPoint = DistanceAndYawBasedCost.computeMidFootPoint(goalNode);
      Point2d nodeMidFootPoint = DistanceAndYawBasedCost.computeMidFootPoint(node);
      double euclideanDistance = nodeMidFootPoint.distance(goalPoint);
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(node.getYaw(), goalNode.getYaw());
      return euclideanDistance + Math.abs(yaw);
   }
}
