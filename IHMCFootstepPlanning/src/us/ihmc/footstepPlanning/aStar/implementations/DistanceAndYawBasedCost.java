package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.footstepPlanning.aStar.FootstepCost;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;

public class DistanceAndYawBasedCost implements FootstepCost
{
   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      double euclideanDistance = startNode.euclideanDistance(endNode);
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(startNode.getYaw(), endNode.getYaw());
      return euclideanDistance + Math.abs(yaw);
   }
}
