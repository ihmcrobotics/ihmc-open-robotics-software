package us.ihmc.robotics.geometry;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class StringStretcher3D
{
   private final Point3D startPoint = new Point3D();
   private final Point3D endPoint = new Point3D();

   private final Line2D tempLine = new Line2D(new Point2D(), new Vector2D(1.0, 0.0));

   private final List<MinMaxPoint3DHolder> minMaxPoints = new ArrayList<>();
   private final RecyclingArrayList<MinMaxPoint3DHolder> recycleablePointHolders = new RecyclingArrayList<>(20, MinMaxPoint3DHolder.class);
   private final RecyclingArrayList<Point3D> recycleableWayPoints = new RecyclingArrayList<>(20, Point3D.class);
   private final List<Point3DBasics> waypoints = new ArrayList<>();
   private final List<MinMaxPoint3DHolder> pointsToInterpolate = new ArrayList<>();

   public void setStartPoint(Point3DReadOnly startPoint)
   {
      this.startPoint.set(startPoint);
   }

   public void setEndPoint(Point3DReadOnly endPoint)
   {
      this.endPoint.set(endPoint);
   }

   /**
    * finds a path between two endpoints that adheres to the min max point constraints.
    * This method returns a list of waypoints which include the start and end points and a waypoint between
    * each pair of min max points.
    * THE WAYPOINTS packed in the waypointListToPack ARE RECYCLED! the points used in the waypoint list live in a pool
    * of Point2D's and will be reused. you must copy the contents of the points inside the waypoint list if you need them to persist,
    * otherwise this method will change the contents of the points
    * @param waypointListToPack
    */
   public void stretchString(List<Point3DBasics> waypointListToPack)
   {
      waypointListToPack.clear();
      pointsToInterpolate.clear();
      recycleableWayPoints.clear();

      findWaypoints(waypoints);

      Point3DBasics leftPoint = startPoint;
      waypointListToPack.add(startPoint);

      int minMaxPointsIndex = 0;
      int waypointsIndex = 0;

      boolean done = false;

      while (!done)
      {
         MinMaxPoint3DHolder minMaxPoint = getMinMaxPoint(minMaxPointsIndex);
         Point3DBasics waypoint = getWaypoint(waypoints, waypointsIndex);

         if ((minMaxPoint == null) && (waypoint == null))
         {
            done = true;
         }
         else if (minMaxPoint == null)
         {
            throw new RuntimeException("Shouldn't get here!");
         }
         else if ((waypoint == minMaxPoint.getMinPoint()) || (waypoint == minMaxPoint.getMaxPoint()))
         {

            for(int i = 0; i < pointsToInterpolate.size(); i++)
            {
               MinMaxPoint3DHolder pointToInterpolate = pointsToInterpolate.get(i);
               Point3D wayPoint = recycleableWayPoints.add();
               interpolate(leftPoint, waypoint, pointToInterpolate.getMinPoint().getX(), wayPoint);
               waypointListToPack.add(wayPoint);
            }

            pointsToInterpolate.clear();

            waypointListToPack.add(waypoint);
            leftPoint = waypoint;
            minMaxPointsIndex++;
            waypointsIndex++;
         }
         else
         {
            pointsToInterpolate.add(minMaxPoint);
            minMaxPointsIndex++;
         }
      }

      for(int i = 0; i < pointsToInterpolate.size(); i++)
      {
         MinMaxPoint3DHolder pointToInterpolate = pointsToInterpolate.get(i);
         Point3D wayPoint = recycleableWayPoints.add();
         interpolate(leftPoint, endPoint, pointToInterpolate.getMinPoint().getX(), wayPoint);
         waypointListToPack.add(wayPoint);
      }

      waypointListToPack.add(endPoint);
   }

   private Point3DBasics getWaypoint(List<Point3DBasics> waypoints, int waypointsIndex)
   {
      if (waypointsIndex < 0) return null;
      if (waypointsIndex >= waypoints.size()) return null;

      Point3DBasics waypoint = waypoints.get(waypointsIndex);
      return waypoint;
   }

   private MinMaxPoint3DHolder getMinMaxPoint(int minMaxPointsIndex)
   {
      if (minMaxPointsIndex < 0) return null;
      if (minMaxPointsIndex >= minMaxPoints.size()) return null;

      MinMaxPoint3DHolder minMaxPoint = minMaxPoints.get(minMaxPointsIndex);
      return minMaxPoint;
   }

   public void findWaypoints(List<Point3DBasics> waypointsToPack)
   {
      waypointsToPack.clear();

      findWaypoints(waypointsToPack, startPoint, endPoint, 0, minMaxPoints.size()-1);
   }

   public void findWaypoints(List<Point3DBasics> waypointsToPack, Point3DReadOnly startPoint, Point3DReadOnly endPoint, int startIndex, int endIndex)
   {
      if (startIndex > endIndex) return;
      if (startIndex < 0)
         return;
      if (endIndex >= minMaxPoints.size())
         return;

      // Find worst min violator
      int worstMinViolatorIndex = findWorstMinViolatorIndex(startPoint, endPoint, startIndex, endIndex);

      if (worstMinViolatorIndex != -1)
      {
         Point3DBasics waypoint = minMaxPoints.get(worstMinViolatorIndex).getMinPoint();
         insertWaypointBeforeEndPoint(waypointsToPack, endPoint, waypoint);

         findWaypoints(waypointsToPack, startPoint, waypoint, startIndex, worstMinViolatorIndex-1);
         findWaypoints(waypointsToPack, waypoint, endPoint, worstMinViolatorIndex + 1, endIndex);
      }

      else
      {
         int worstMaxViolatorIndex = findWorstMaxViolatorIndex(startPoint, endPoint, startIndex, endIndex);

         if (worstMaxViolatorIndex != -1)
         {
            Point3DBasics waypoint = minMaxPoints.get(worstMaxViolatorIndex).getMaxPoint();
            insertWaypointBeforeEndPoint(waypointsToPack, endPoint, waypoint);

            findWaypoints(waypointsToPack, startPoint, waypoint, startIndex, worstMaxViolatorIndex-1);
            findWaypoints(waypointsToPack, waypoint, endPoint, worstMaxViolatorIndex + 1, endIndex);
         }
      }

   }

   private void insertWaypointBeforeEndPoint(List<Point3DBasics> waypointsToPack, Point3DReadOnly endPoint, Point3DBasics waypoint)
   {
      if (endPoint == this.endPoint)
      {
         waypointsToPack.add(waypoint);
      }
      else // Add before endPoint
      {
         waypointsToPack.add(waypointsToPack.indexOf(endPoint), waypoint);
      }
   }


   private void interpolate(Point3DReadOnly startPoint, Point3DReadOnly endPoint, double x, Point3DBasics midPointToPack)
   {
      double startX = startPoint.getX();
      double endX = endPoint.getX();

      double startY = startPoint.getY();
      double endY = endPoint.getY();

      double startZ = startPoint.getZ();
      double endZ = endPoint.getZ();

      double percentX = (x - startX) / (endX - startX);
      double y = startY + percentX * (endY - startY);
      double z = startZ + percentX * (endZ - startZ);

      midPointToPack.set(x, y, z);
   }

   public void addMinMaxPoints(Point3DBasics minPoint, Point3DBasics maxPoint)
   {
      double x = minPoint.getX();
      if (Math.abs(x - maxPoint.getX()) > 1e-7)
         throw new RuntimeException();
      if (minPoint.getY() > maxPoint.getY()) throw new RuntimeException();

      if (x < startPoint.getX())
         throw new RuntimeException();
      if (x > endPoint.getX())
         throw new RuntimeException();

      int i = 0;
      while ((minMaxPoints.size() > i) && (x > minMaxPoints.get(i).getMinPoint().getX()))
      {
         i++;
      }

      MinMaxPoint3DHolder pointHolder = recycleablePointHolders.add();
      pointHolder.setMaxPoint(maxPoint);
      pointHolder.setMinPoint(minPoint);
      this.minMaxPoints.add(i, pointHolder);
   }



   private int findWorstMinViolatorIndex(Point3DReadOnly startPoint, Point3DReadOnly endPoint, int startIndex, int endIndex)
   {
      tempLine.set(startPoint.getX(), startPoint.getZ(), endPoint.getX(), endPoint.getZ());

      int returnIndex = -1;
      double worstViolation = Double.NEGATIVE_INFINITY;

      for (int i = startIndex; i <= endIndex; i++)
      {
         MinMaxPoint3DHolder minMaxPoint = getMinMaxPoint(i);
         Point3DBasics minPoint = minMaxPoint.getMinPoint();
         boolean isPointOnLeftSide = EuclidGeometryTools.isPoint2DOnSideOfLine2D(minPoint.getX(), minPoint.getZ(), tempLine.getPoint(), tempLine.getDirection(), true);
         if (isPointOnLeftSide)
         {
            double distance = EuclidGeometryTools.distanceFromPoint2DToLine2D(minPoint.getX(), minPoint.getZ(), tempLine.getPoint(), tempLine.getDirection());
            if (distance > worstViolation)
            {
               worstViolation = distance;
               returnIndex = i;
            }
         }
      }

      return returnIndex;
   }

   private int findWorstMaxViolatorIndex(Point3DReadOnly startPoint, Point3DReadOnly endPoint, int startIndex, int endIndex)
   {
      tempLine.set(startPoint.getX(), startPoint.getZ(), endPoint.getX(), endPoint.getZ());

      int returnIndex = -1;
      double worstViolation = Double.NEGATIVE_INFINITY;

      for (int i = startIndex; i <= endIndex; i++)
      {
         MinMaxPoint3DHolder minMaxPoint = getMinMaxPoint(i);
         Point3DBasics maxPoint = minMaxPoint.getMaxPoint();
         boolean isPointOnRightSide = EuclidGeometryTools.isPoint2DOnSideOfLine2D(maxPoint.getX(), maxPoint.getZ(), tempLine.getPoint(), tempLine.getDirection(), false);

         if (isPointOnRightSide)
         {
            double distance = EuclidGeometryTools.distanceFromPoint2DToLine2D(maxPoint.getX(), maxPoint.getZ(), tempLine.getPoint(), tempLine.getDirection());
            if (distance > worstViolation)
            {
               worstViolation = distance;
               returnIndex = i;
            }
         }
      }

      return returnIndex;
   }


   public void reset()
   {
      minMaxPoints.clear();
      recycleablePointHolders.clear();
   }
}
