package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class StringStretcher2d
{
   private final Point2D startPoint = new Point2D();
   private final Point2D endPoint = new Point2D();

   private final RecyclingArrayList<MinMaxPointHolder> minMaxPool = new RecyclingArrayList<>(20, MinMaxPointHolder.class);
   private final RecyclingArrayList<Point2D> waypointPool = new RecyclingArrayList<>(20, Point2D.class);

   private final List<MinMaxPointHolder> minMaxPoints = new ArrayList<>();
   private final List<Point2DBasics> waypoints = new ArrayList<>();
   private final List<MinMaxPointHolder> pointsToInterpolate = new ArrayList<MinMaxPointHolder>();

   public void setStartPoint(Point2D startPoint)
   {
      setStartPoint(startPoint.getX(), startPoint.getY());
   }

   public void setStartPoint(double x, double y)
   {
      this.startPoint.set(x, y);
   }

   public void setEndPoint(Point2DReadOnly endPoint)
   {
      setEndPoint(endPoint.getX(), endPoint.getY());
   }

   public void setEndPoint(double x, double y)
   {
      this.endPoint.set(x, y);
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
   public void stretchString(List<Point2DBasics> waypointListToPack)
   {
      waypointListToPack.clear();
      pointsToInterpolate.clear();
      waypointPool.clear();

      findWaypoints(waypoints);

      Point2DBasics leftPoint = startPoint;
      waypointListToPack.add(startPoint);

      int minMaxPointsIndex = 0;
      int waypointsIndex = 0;

      boolean done = false;

      while (!done)
      {
         MinMaxPointHolder minMaxPoint = getMinMaxPoint(minMaxPointsIndex);
         Point2DBasics waypoint = getWaypoint(waypoints, waypointsIndex);

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

            for (int i = 0; i < pointsToInterpolate.size(); i++)
            {
               MinMaxPointHolder pointToInterpolate = pointsToInterpolate.get(i);
               Point2DBasics wayPoint = waypointPool.add();
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

      for (int i = 0; i < pointsToInterpolate.size(); i++)
      {
         MinMaxPointHolder pointToInterpolate = pointsToInterpolate.get(i);
         Point2D wayPoint = waypointPool.add();
         interpolate(leftPoint, endPoint, pointToInterpolate.getMinPoint().getX(), wayPoint);
         waypointListToPack.add(wayPoint);
      }

      waypointListToPack.add(endPoint);
   }

   private MinMaxPointHolder getMinMaxPoint(int minMaxPointsIndex)
   {
      if (minMaxPointsIndex < 0)
         return null;
      if (minMaxPointsIndex >= minMaxPoints.size())
         return null;

      return minMaxPoints.get(minMaxPointsIndex);
   }

   private Point2DBasics getWaypoint(List<Point2DBasics> waypoints, int waypointsIndex)
   {
      if (waypointsIndex < 0)
         return null;
      if (waypointsIndex >= waypoints.size())
         return null;

      return waypoints.get(waypointsIndex);
   }

   public void findWaypoints(List<Point2DBasics> waypointsToPack)
   {
      waypointsToPack.clear();

      findWaypoints(waypointsToPack, startPoint, endPoint, 0, minMaxPoints.size() - 1);
   }

   public void findWaypoints(List<Point2DBasics> waypointsToPack, Point2DReadOnly startPoint, Point2DReadOnly endPoint, int startIndex, int endIndex)
   {
      if (startIndex > endIndex)
         return;
      if (startIndex < 0)
         return;
      if (endIndex >= minMaxPoints.size())
         return;

      // Find worst min violator
      int worstMinViolatorIndex = findWorstMinViolatorIndex(startPoint, endPoint, startIndex, endIndex);

      if (worstMinViolatorIndex != -1)
      {
         Point2DBasics waypoint = minMaxPoints.get(worstMinViolatorIndex).getMinPoint();
         insertWaypointBeforeEndPoint(waypointsToPack, endPoint, waypoint);

         findWaypoints(waypointsToPack, startPoint, waypoint, startIndex, worstMinViolatorIndex - 1);
         findWaypoints(waypointsToPack, waypoint, endPoint, worstMinViolatorIndex + 1, endIndex);
      }

      else
      {
         int worstMaxViolatorIndex = findWorstMaxViolatorIndex(startPoint, endPoint, startIndex, endIndex);

         if (worstMaxViolatorIndex != -1)
         {
            Point2DBasics waypoint = minMaxPoints.get(worstMaxViolatorIndex).getMaxPoint();
            insertWaypointBeforeEndPoint(waypointsToPack, endPoint, waypoint);

            findWaypoints(waypointsToPack, startPoint, waypoint, startIndex, worstMaxViolatorIndex - 1);
            findWaypoints(waypointsToPack, waypoint, endPoint, worstMaxViolatorIndex + 1, endIndex);
         }
      }
   }

   private void insertWaypointBeforeEndPoint(List<Point2DBasics> waypointsToPack, Point2DReadOnly endPoint, Point2DBasics waypoint)
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

   public void addMinMaxPoints(Point2DReadOnly minPoint, Point2DReadOnly maxPoint)
   {
      addMinMaxPoints(minPoint.getX(), minPoint.getY(), maxPoint.getX(), maxPoint.getY());
   }

   public void addMinMaxPoints(double minPointX, double minPointY, double maxPointX, double maxPointY)
   {
      double x = minPointX;
      if (Math.abs(x - maxPointX) > 1e-7)
         throw new RuntimeException("Min X " + x + " and Max X " + maxPointX + " aren't far enough apart.");
      if (minPointY > maxPointY)
         throw new RuntimeException("Min Y " + minPointY + " is greater than Max Y " + maxPointY + ".");

      if (x < startPoint.getX())
         throw new RuntimeException("Min X " + x + "  is less than the start point " + startPoint.getX() + ".");
      if (x > endPoint.getX())
         throw new RuntimeException("Min X " + x + "  is greater than the end point " + endPoint.getX() + ".");

      int i = 0;
      while ((minMaxPoints.size() > i) && (x > minMaxPoints.get(i).getMinPoint().getX()))
      {
         i++;
      }

      MinMaxPointHolder pointHolder = minMaxPool.add();
      pointHolder.setMaxPoint(maxPointX, maxPointY);
      pointHolder.setMinPoint(minPointX, minPointY);
      this.minMaxPoints.add(i, pointHolder);
   }

   public Point2DReadOnly findWorstMinViolator(Point2D startPoint, Point2D endPoint)
   {
      int worstViolatorIndex = findWorstMinViolatorIndex(startPoint, endPoint, 0, minMaxPoints.size() - 1);
      if (worstViolatorIndex == -1)
         return null;

      MinMaxPointHolder minMaxPoint = getMinMaxPoint(worstViolatorIndex);

      return minMaxPoint.getMinPoint();
   }

   public Point2DReadOnly findWorstMaxViolator(Point2D startPoint, Point2D endPoint)
   {
      int worstViolatorIndex = findWorstMaxViolatorIndex(startPoint, endPoint, 0, minMaxPoints.size() - 1);
      if (worstViolatorIndex == -1)
         return null;

      MinMaxPointHolder minMaxPoint = getMinMaxPoint(worstViolatorIndex);

      return minMaxPoint.getMaxPoint();
   }

   private int findWorstMinViolatorIndex(Point2DReadOnly startPoint, Point2DReadOnly endPoint, int startIndex, int endIndex)
   {
      int returnIndex = -1;
      double worstViolation = 0.0;

      for (int i = startIndex; i <= endIndex; i++)
      {
         Point2DReadOnly minPoint = minMaxPoints.get(i).getMinPoint();
         double interpolatedHeightAtPoint = interpolate(startPoint, endPoint, minPoint.getX());
         double minPointY = minPoint.getY();
         if (interpolatedHeightAtPoint - minPointY < worstViolation)
         {
            worstViolation = interpolatedHeightAtPoint - minPointY;
            returnIndex = i;
         }
      }

      return returnIndex;
   }

   private int findWorstMaxViolatorIndex(Point2DReadOnly startPoint, Point2DReadOnly endPoint, int startIndex, int endIndex)
   {
      int returnIndex = -1;
      double worstViolation = 0.0;

      for (int i = startIndex; i <= endIndex; i++)
      {
         Point2DReadOnly maxPoint = minMaxPoints.get(i).getMaxPoint();
         double interpolatedHeightAtPoint = interpolate(startPoint, endPoint, maxPoint.getX());
         double maxPointY = maxPoint.getY();
         if (maxPointY - interpolatedHeightAtPoint < worstViolation)
         {
            worstViolation = maxPointY - interpolatedHeightAtPoint;
            returnIndex = i;
         }
      }

      return returnIndex;
   }

   public MinMaxPointHolder findMinMaxPoints(double x)
   {
      for (int i = 0; i < minMaxPoints.size(); i++)
      {
         MinMaxPointHolder minMaxPoint = minMaxPoints.get(i);
         if (Math.abs(x - minMaxPoint.getMinPoint().getX()) < 1e-7)
            return minMaxPoint;
      }

      return null;
   }

   public void reset()
   {
      minMaxPoints.clear();
      minMaxPool.clear();
   }

   private static void interpolate(Point2DReadOnly startPoint, Point2DReadOnly endPoint, double x, Point2DBasics midPointToPack)
   {
      midPointToPack.set(x, interpolate(startPoint, endPoint, x));
   }

   private static double interpolate(Point2DReadOnly startPoint, Point2DReadOnly endPoint, double x)
   {
      double startX = startPoint.getX();
      double endX = endPoint.getX();

      double percentX = (x - startX) / (endX - startX);

      return EuclidCoreTools.interpolate(startPoint.getY(), endPoint.getY(), percentX);
   }
}
