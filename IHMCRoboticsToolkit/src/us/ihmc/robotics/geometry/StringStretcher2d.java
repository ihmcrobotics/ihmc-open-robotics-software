package us.ihmc.robotics.geometry;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import java.util.ArrayList;
import java.util.List;

public class StringStretcher2d
{
   private final Point2d startPoint = new Point2d();
   private final Point2d endPoint = new Point2d();

   private final Line2d tempLine = new Line2d(new Point2d(), new Vector2d(1.0, 0.0));
   private final Vector2d tempVectorA = new Vector2d();
   private final Vector2d tempVectorB = new Vector2d();

   private final ArrayList<Point2d[]> minMaxPoints = new ArrayList<Point2d[]>();

   public void setStartPoint(Point2d startPoint)
   {
      this.startPoint.set(startPoint);
   }

   public void setEndPoint(Point2d endPoint)
   {
      this.endPoint.set(endPoint);
   }

   public List<Point2d> stretchString()
   {
      ArrayList<Point2d> ret = new ArrayList<Point2d>();

      ArrayList<Point2d> waypoints = new ArrayList<Point2d>();
      findWaypoints(waypoints);

      Point2d leftPoint = startPoint;
      ArrayList<Point2d[]> pointsToInterpolate = new ArrayList<Point2d[]>();

      ret.add(startPoint);

      int minMaxPointsIndex = 0;
      int waypointsIndex = 0;

      boolean done = false;

      while (!done)
      {
         Point2d[] minMaxPoint = getMinMaxPoint(minMaxPointsIndex);
         Point2d waypoint = getWaypoint(waypoints, waypointsIndex);

         if ((minMaxPoint == null) && (waypoint == null))
         {
            done = true;
         }
         else if (minMaxPoint == null)
         {
            throw new RuntimeException("Shouldn't get here!");
         }
         else if ((waypoint == minMaxPoint[0]) || (waypoint == minMaxPoint[1]))
         {
            for (Point2d[] pointToInterpolate : pointsToInterpolate)
            {
               ret.add(interpolate(leftPoint, waypoint, pointToInterpolate[0].getX()));
            }

            pointsToInterpolate.clear();

            ret.add(waypoint);
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

      for (Point2d[] pointToInterpolate : pointsToInterpolate)
      {
         ret.add(interpolate(leftPoint, endPoint, pointToInterpolate[0].getX()));
      }
      
      ret.add(endPoint);

      return ret;
   }

   private Point2d getWaypoint(ArrayList<Point2d> waypoints, int waypointsIndex)
   {
      if (waypointsIndex < 0) return null;
      if (waypointsIndex >= waypoints.size()) return null;
      
      Point2d waypoint = waypoints.get(waypointsIndex);
      return waypoint;
   }

   private Point2d[] getMinMaxPoint(int minMaxPointsIndex)
   {
      if (minMaxPointsIndex < 0) return null;
      if (minMaxPointsIndex >= minMaxPoints.size()) return null;
      
      Point2d[] minMaxPoint = minMaxPoints.get(minMaxPointsIndex);
      return minMaxPoint;
   }

   public void findWaypoints(ArrayList<Point2d> waypointsToPack)
   {
      waypointsToPack.clear();

      findWaypoints(waypointsToPack, startPoint, endPoint, 0, minMaxPoints.size()-1);
   }

   public void findWaypoints(ArrayList<Point2d> waypointsToPack, Point2d startPoint, Point2d endPoint, int startIndex, int endIndex)
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
         Point2d waypoint = minMaxPoints.get(worstMinViolatorIndex)[0];
         insertWaypointBeforeEndPoint(waypointsToPack, endPoint, waypoint);
         
         findWaypoints(waypointsToPack, startPoint, waypoint, startIndex, worstMinViolatorIndex-1);
         findWaypoints(waypointsToPack, waypoint, endPoint, worstMinViolatorIndex + 1, endIndex);
      }

      else
      {
         int worstMaxViolatorIndex = findWorstMaxViolatorIndex(startPoint, endPoint, startIndex, endIndex);

         if (worstMaxViolatorIndex != -1)
         {
            Point2d waypoint = minMaxPoints.get(worstMaxViolatorIndex)[1];
            insertWaypointBeforeEndPoint(waypointsToPack, endPoint, waypoint);
            
            findWaypoints(waypointsToPack, startPoint, waypoint, startIndex, worstMaxViolatorIndex-1);
            findWaypoints(waypointsToPack, waypoint, endPoint, worstMaxViolatorIndex + 1, endIndex);
         }
      }

   }

   private void insertWaypointBeforeEndPoint(ArrayList<Point2d> waypointsToPack, Point2d endPoint, Point2d waypoint)
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


   private Point2d interpolate(Point2d startPoint, Point2d endPoint, double x)
   {
      double startX = startPoint.getX();
      double endX = endPoint.getX();

      double startY = startPoint.getY();
      double endY = endPoint.getY();

      double percentX = (x - startX) / (endX - startX);
      double y = startY + percentX * (endY - startY);

      return new Point2d(x, y);
   }

   public void addMinMaxPoints(Point2d minPoint, Point2d maxPoint)
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
      while ((minMaxPoints.size() > i) && (x > minMaxPoints.get(i)[0].getX()))
      {
         i++;
      }

      this.minMaxPoints.add(i, new Point2d[] {minPoint, maxPoint});
   }


   public Point2d findWorstMinViolator(Point2d startPoint, Point2d endPoint)
   {
      int worstViolatorIndex = findWorstMinViolatorIndex(startPoint, endPoint, 0, minMaxPoints.size()-1);
      if (worstViolatorIndex == -1)
         return null;

      Point2d[] minMaxPoint = getMinMaxPoint(worstViolatorIndex);

      return minMaxPoint[0];
   }

   private int findWorstMinViolatorIndex(Point2d startPoint, Point2d endPoint, int startIndex, int endIndex)
   {
      tempLine.set(startPoint, endPoint);

      int returnIndex = -1;
      double worstViolation = Double.NEGATIVE_INFINITY;

      for (int i = startIndex; i <= endIndex; i++)
      {
         Point2d[] minMaxPoint = getMinMaxPoint(i);
         Point2d minPoint = minMaxPoint[0];
         if (tempLine.isPointOnLeftSideOfLine(minPoint))
         {
            double distance = tempLine.distance(minPoint);
            if (distance > worstViolation)
            {
               worstViolation = distance;
               returnIndex = i;
            }
         }
      }

      return returnIndex;
   }

   public Point2d findWorstMaxViolator(Point2d startPoint, Point2d endPoint)
   {
      int worstViolatorIndex = findWorstMaxViolatorIndex(startPoint, endPoint, 0, minMaxPoints.size()-1);
      if (worstViolatorIndex == -1)
         return null;

      Point2d[] minMaxPoint = getMinMaxPoint(worstViolatorIndex);

      return minMaxPoint[1];
   }

   private int findWorstMaxViolatorIndex(Point2d startPoint, Point2d endPoint, int startIndex, int endIndex)
   {
      tempLine.set(startPoint, endPoint);

      int returnIndex = -1;
      double worstViolation = Double.NEGATIVE_INFINITY;

      for (int i = startIndex; i <= endIndex; i++)
      {
         Point2d[] minMaxPoint = getMinMaxPoint(i);

         Point2d maxPoint = minMaxPoint[1];
         if (tempLine.isPointOnRightSideOfLine(maxPoint))
         {
            double distance = tempLine.distance(maxPoint);
            if (distance > worstViolation)
            {
               worstViolation = distance;
               returnIndex = i;
            }
         }
      }

      return returnIndex;
   }

   public Point2d[] findMinMaxPoints(double x)
   {
      for (Point2d[] minMaxPoint : minMaxPoints)
      {
         if (Math.abs(x - minMaxPoint[0].getX()) < 1e-7) return minMaxPoint;
      }
      
      return null;
   }
}
