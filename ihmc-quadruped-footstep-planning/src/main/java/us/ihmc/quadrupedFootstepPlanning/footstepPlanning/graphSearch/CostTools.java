package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;

public class CostTools
{
   public static double computeReferenceYaw(Point2DReadOnly startPoint, double startYaw, FootstepNode goalNode, double proximity)
   {
      Point2DReadOnly goalCenterPoint = goalNode.getOrComputeXGaitCenterPoint();

      double yawMultiplier = computeDistanceToGoalScalar(startPoint.getX(), startPoint.getY(), goalNode, proximity);

      double pathHeading = Math.atan2(goalCenterPoint.getY() - startPoint.getY(), goalCenterPoint.getX() - startPoint.getX());
      pathHeading = AngleTools.trimAngleMinusPiToPi(pathHeading);

      double angleDifference = AngleTools.computeAngleDifferenceMinusPiToPi(startYaw, pathHeading);
      double modifiedPathHeading = pathHeading;
      if (Math.abs(angleDifference) > Math.PI / 2.0) // greater than 90 degrees, so go backwards
         modifiedPathHeading = AngleTools.trimAngleMinusPiToPi(modifiedPathHeading + Math.PI);


      double referenceHeading = yawMultiplier * modifiedPathHeading;
      referenceHeading += (1.0 - yawMultiplier) * goalNode.getStepYaw();
      return AngleTools.trimAngleMinusPiToPi(referenceHeading);
   }

   public static double computeDistanceToGoalScalar(double x, double y, FootstepNode goalNode, double proximity)
   {
      Point2DReadOnly goalCenter = goalNode.getOrComputeXGaitCenterPoint();
      double distanceToGoal = EuclidCoreTools.norm(x - goalCenter.getX(), y - goalCenter.getY());

      double minimumBlendDistance = 0.75 * proximity;
      double maximumBlendDistance = 1.25 * proximity;

      double multiplier;
      if (distanceToGoal < minimumBlendDistance)
         multiplier = 0.0;
      else if(distanceToGoal > maximumBlendDistance)
         multiplier = 1.0;
      else
         multiplier = (distanceToGoal - minimumBlendDistance) / (maximumBlendDistance - minimumBlendDistance);

      return multiplier;
   }
}
