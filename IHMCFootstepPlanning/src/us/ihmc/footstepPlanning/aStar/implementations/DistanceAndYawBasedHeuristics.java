package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.footstepPlanning.aStar.CostToGoHeuristics;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;

public class DistanceAndYawBasedHeuristics extends CostToGoHeuristics
{
   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      double euclideanDistance = node.euclideanDistance(goalNode);
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(node.getYaw(), goalNode.getYaw());
      return euclideanDistance + Math.abs(yaw);
   }
}
