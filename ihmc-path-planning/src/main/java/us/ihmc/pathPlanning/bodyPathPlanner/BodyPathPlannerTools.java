package us.ihmc.pathPlanning.bodyPathPlanner;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.AngleTools;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class BodyPathPlannerTools
{
   public static double calculateHeading(Point2DReadOnly startPosition, Point2DReadOnly endPosition)
   {
      double deltaX = endPosition.getX() - startPosition.getX();
      double deltaY = endPosition.getY() - startPosition.getY();

      return calculateHeading(deltaX, deltaY);
   }

   public static double calculateHeading(Point3DReadOnly startPosition, Point3DReadOnly endPosition)
   {
      double deltaX = endPosition.getX() - startPosition.getX();
      double deltaY = endPosition.getY() - startPosition.getY();

      return calculateHeading(deltaX, deltaY);
   }

   public static double calculateHeading(Vector2DReadOnly direction)
   {
      return calculateHeading(direction.getX(), direction.getY());
   }

   public static double calculateHeading(double deltaX, double deltaY)
   {
      return Math.atan2(deltaY, deltaX);
   }

   public static int findClosestPointAlongPath(List<? extends Pose3DReadOnly> bodyPathPlan, Point3DReadOnly nearbyPoint, Point3DBasics closestPointToPack)
   {
      closestPointToPack.set(bodyPathPlan.get(0).getPosition());
      double closestDistance = closestPointToPack.distance(nearbyPoint);
      int closestSegmentIndex = 0;
      for (int i = 0; i < bodyPathPlan.size() - 1; i++)
      {
         LogTools.trace("Finding closest point along body path. Segment: {}, closestDistance: {}", i, closestDistance);
         LineSegment3D lineSegment = new LineSegment3D();
         lineSegment.set(bodyPathPlan.get(i).getPosition(), bodyPathPlan.get(i + 1).getPosition());

         Point3D closestPointOnBodyPathSegment = new Point3D();
         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegment.getFirstEndpoint(),
                                                                     lineSegment.getSecondEndpoint(),
                                                                     nearbyPoint,
                                                                     nearbyPoint,
                                                                     closestPointOnBodyPathSegment,
                                                                     new Point3D()); // TODO find a better way to do this

         double distance = closestPointOnBodyPathSegment.distance(nearbyPoint);
         if (distance < closestDistance)
         {
            closestPointToPack.set(closestPointOnBodyPathSegment);
            closestDistance = distance;
            closestSegmentIndex = i;
         }
      }
      LogTools.trace("closestPointAlongPath: {}, closestDistance: {}, closestLineSegmentIndex: {}", closestPointToPack, closestDistance, closestSegmentIndex);

      return closestSegmentIndex;
   }
}
