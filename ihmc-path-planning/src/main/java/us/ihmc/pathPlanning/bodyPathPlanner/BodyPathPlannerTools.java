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

   /**
    * @return segment index closest point is on
    */
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
         lineSegment.orthogonalProjection(nearbyPoint, closestPointOnBodyPathSegment);

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

   public static int movePointAlongBodyPath(List<? extends Pose3DReadOnly> bodyPathPlan,
                                            Point3DReadOnly startPoint,
                                            Point3DBasics movedPointToPack,
                                            double distance)
   {
      Point3D closestPointOnPlan = new Point3D();
      int segmentToStartOn = findClosestPointAlongPath(bodyPathPlan, startPoint, closestPointOnPlan);

      return movePointAlongBodyPath(bodyPathPlan, closestPointOnPlan, movedPointToPack, segmentToStartOn, distance);
   }

   /**
    * Move point along body path plan by a distance.
    *
    * @return segment index moved point ends up on
    */
   public static int movePointAlongBodyPath(List<? extends Pose3DReadOnly> bodyPathPlan,
                                             Point3DReadOnly pointOnBodyPath,
                                             Point3DBasics movedPointToPack,
                                             int segmentToStartOn,
                                             double distance)
   {
      double moveAmountToGo = distance;
      Point3D previousComparisonPoint = new Point3D(pointOnBodyPath);
      Point3D endOfSegment = new Point3D();
      int segmentIndexOfGoal = segmentToStartOn;
      movedPointToPack.set(pointOnBodyPath);
      for (int i = segmentToStartOn; i < bodyPathPlan.size() - 1 && moveAmountToGo > 0.0; i++)
      {
         endOfSegment.set((bodyPathPlan.get(i + 1).getPosition()));

         double distanceToEndOfSegment = endOfSegment.distance(previousComparisonPoint);
         LogTools.trace("Evaluating segment {}, moveAmountToGo: {}, distanceToEndOfSegment: {}", i, moveAmountToGo, distanceToEndOfSegment);

         if (distanceToEndOfSegment < moveAmountToGo)
         {
            previousComparisonPoint.set(bodyPathPlan.get(i + 1).getPosition());
            moveAmountToGo -= distanceToEndOfSegment;
         }
         else
         {
            movedPointToPack.interpolate(previousComparisonPoint, endOfSegment, moveAmountToGo / distanceToEndOfSegment);
            segmentIndexOfGoal = i;
            break;
         }

         movedPointToPack.set(previousComparisonPoint);
         segmentIndexOfGoal = i;
      }
      LogTools.trace("previousComparisonPoint: {}, goalPoint: {}", previousComparisonPoint, movedPointToPack);

      return segmentIndexOfGoal;
   }
}
