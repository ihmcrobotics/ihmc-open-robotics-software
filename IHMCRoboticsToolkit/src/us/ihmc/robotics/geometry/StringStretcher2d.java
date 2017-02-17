package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class StringStretcher2d
{
   private final Point2D startPoint = new Point2D();
   private final Point2D endPoint = new Point2D();

   private final Line2d tempLine = new Line2d(new Point2D(), new Vector2D(1.0, 0.0));

   private final ArrayList<Point2D[]> minMaxPoints = new ArrayList<Point2D[]>();

   public void setStartPoint(Point2D startPoint)
   {
      this.startPoint.set(startPoint);
   }

   public void setEndPoint(Point2D endPoint)
   {
      this.endPoint.set(endPoint);
   }

   public List<Point2D> stretchString()
   {
      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      ArrayList<Point2D> waypoints = new ArrayList<Point2D>();
      findWaypoints(waypoints);

      Point2D leftPoint = startPoint;
      ArrayList<Point2D[]> pointsToInterpolate = new ArrayList<Point2D[]>();

      ret.add(startPoint);

      int minMaxPointsIndex = 0;
      int waypointsIndex = 0;

      boolean done = false;

      while (!done)
      {
         Point2D[] minMaxPoint = getMinMaxPoint(minMaxPointsIndex);
         Point2D waypoint = getWaypoint(waypoints, waypointsIndex);

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
            for (Point2D[] pointToInterpolate : pointsToInterpolate)
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

      for (Point2D[] pointToInterpolate : pointsToInterpolate)
      {
         ret.add(interpolate(leftPoint, endPoint, pointToInterpolate[0].getX()));
      }
      
      ret.add(endPoint);

      return ret;
   }

   private Point2D getWaypoint(ArrayList<Point2D> waypoints, int waypointsIndex)
   {
      if (waypointsIndex < 0) return null;
      if (waypointsIndex >= waypoints.size()) return null;
      
      Point2D waypoint = waypoints.get(waypointsIndex);
      return waypoint;
   }

   private Point2D[] getMinMaxPoint(int minMaxPointsIndex)
   {
      if (minMaxPointsIndex < 0) return null;
      if (minMaxPointsIndex >= minMaxPoints.size()) return null;
      
      Point2D[] minMaxPoint = minMaxPoints.get(minMaxPointsIndex);
      return minMaxPoint;
   }

   public void findWaypoints(ArrayList<Point2D> waypointsToPack)
   {
      waypointsToPack.clear();

      findWaypoints(waypointsToPack, startPoint, endPoint, 0, minMaxPoints.size()-1);
   }

   public void findWaypoints(ArrayList<Point2D> waypointsToPack, Point2D startPoint, Point2D endPoint, int startIndex, int endIndex)
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
         Point2D waypoint = minMaxPoints.get(worstMinViolatorIndex)[0];
         insertWaypointBeforeEndPoint(waypointsToPack, endPoint, waypoint);
         
         findWaypoints(waypointsToPack, startPoint, waypoint, startIndex, worstMinViolatorIndex-1);
         findWaypoints(waypointsToPack, waypoint, endPoint, worstMinViolatorIndex + 1, endIndex);
      }

      else
      {
         int worstMaxViolatorIndex = findWorstMaxViolatorIndex(startPoint, endPoint, startIndex, endIndex);

         if (worstMaxViolatorIndex != -1)
         {
            Point2D waypoint = minMaxPoints.get(worstMaxViolatorIndex)[1];
            insertWaypointBeforeEndPoint(waypointsToPack, endPoint, waypoint);
            
            findWaypoints(waypointsToPack, startPoint, waypoint, startIndex, worstMaxViolatorIndex-1);
            findWaypoints(waypointsToPack, waypoint, endPoint, worstMaxViolatorIndex + 1, endIndex);
         }
      }

   }

   private void insertWaypointBeforeEndPoint(ArrayList<Point2D> waypointsToPack, Point2D endPoint, Point2D waypoint)
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


   private Point2D interpolate(Point2D startPoint, Point2D endPoint, double x)
   {
      double startX = startPoint.getX();
      double endX = endPoint.getX();

      double startY = startPoint.getY();
      double endY = endPoint.getY();

      double percentX = (x - startX) / (endX - startX);
      double y = startY + percentX * (endY - startY);

      return new Point2D(x, y);
   }

   public void addMinMaxPoints(Point2D minPoint, Point2D maxPoint)
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

      this.minMaxPoints.add(i, new Point2D[] {minPoint, maxPoint});
   }


   public Point2D findWorstMinViolator(Point2D startPoint, Point2D endPoint)
   {
      int worstViolatorIndex = findWorstMinViolatorIndex(startPoint, endPoint, 0, minMaxPoints.size()-1);
      if (worstViolatorIndex == -1)
         return null;

      Point2D[] minMaxPoint = getMinMaxPoint(worstViolatorIndex);

      return minMaxPoint[0];
   }

   private int findWorstMinViolatorIndex(Point2D startPoint, Point2D endPoint, int startIndex, int endIndex)
   {
      tempLine.set(startPoint, endPoint);

      int returnIndex = -1;
      double worstViolation = Double.NEGATIVE_INFINITY;

      for (int i = startIndex; i <= endIndex; i++)
      {
         Point2D[] minMaxPoint = getMinMaxPoint(i);
         Point2D minPoint = minMaxPoint[0];
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

   public Point2D findWorstMaxViolator(Point2D startPoint, Point2D endPoint)
   {
      int worstViolatorIndex = findWorstMaxViolatorIndex(startPoint, endPoint, 0, minMaxPoints.size()-1);
      if (worstViolatorIndex == -1)
         return null;

      Point2D[] minMaxPoint = getMinMaxPoint(worstViolatorIndex);

      return minMaxPoint[1];
   }

   private int findWorstMaxViolatorIndex(Point2D startPoint, Point2D endPoint, int startIndex, int endIndex)
   {
      tempLine.set(startPoint, endPoint);

      int returnIndex = -1;
      double worstViolation = Double.NEGATIVE_INFINITY;

      for (int i = startIndex; i <= endIndex; i++)
      {
         Point2D[] minMaxPoint = getMinMaxPoint(i);

         Point2D maxPoint = minMaxPoint[1];
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

   public Point2D[] findMinMaxPoints(double x)
   {
      for (Point2D[] minMaxPoint : minMaxPoints)
      {
         if (Math.abs(x - minMaxPoint[0].getX()) < 1e-7) return minMaxPoint;
      }
      
      return null;
   }

   public void reset()
   {
      minMaxPoints.clear();
   }
}
