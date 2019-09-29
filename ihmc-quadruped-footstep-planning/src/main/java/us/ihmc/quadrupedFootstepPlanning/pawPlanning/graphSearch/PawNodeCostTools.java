package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.robotics.geometry.AngleTools;

public class PawNodeCostTools
{
   public static double computeReferenceYaw(Point2DReadOnly startPoint, double startYaw, FramePose3DReadOnly goalPose, double proximity)
   {
      Point3DReadOnly goalCenterPoint = goalPose.getPosition();

      double yawMultiplier = computeDistanceToGoalScalar(startPoint.getX(), startPoint.getY(), goalPose, proximity);

      double pathHeading = Math.atan2(goalCenterPoint.getY() - startPoint.getY(), goalCenterPoint.getX() - startPoint.getX());
      pathHeading = AngleTools.trimAngleMinusPiToPi(pathHeading);

      double angleDifference = AngleTools.computeAngleDifferenceMinusPiToPi(startYaw, pathHeading);
      double modifiedPathHeading = pathHeading;
      if (Math.abs(angleDifference) > Math.PI / 2.0) // greater than 90 degrees, so go backwards
         modifiedPathHeading = AngleTools.trimAngleMinusPiToPi(modifiedPathHeading + Math.PI);


      double referenceHeading = yawMultiplier * modifiedPathHeading;
      referenceHeading += (1.0 - yawMultiplier) * goalPose.getYaw();
      return AngleTools.trimAngleMinusPiToPi(referenceHeading);
   }

   public static double computeDistanceToGoalScalar(double x, double y, FramePose3DReadOnly goalPose, double proximity)
   {
      Point3DReadOnly goalCenter = goalPose.getPosition();
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
