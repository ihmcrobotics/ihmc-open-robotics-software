package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class StringStretcher2d
{
   private final Point2D startPoint = new Point2D();
   private final Point2D endPoint = new Point2D();

   private final Line2D tempLine = new Line2D(new Point2D(), new Vector2D(1.0, 0.0));

   private final ArrayList<MinMaxPointHolder> minMaxPoints = new ArrayList<MinMaxPointHolder>();
   private final RecyclingArrayList<MinMaxPointHolder> recycleablePointHolders = new RecyclingArrayList<>(20, MinMaxPointHolder.class);
   private final RecyclingArrayList<Point2D> recycleableWayPoints = new RecyclingArrayList<>(20, Point2D.class);
   private final ArrayList<Point2D> waypoints = new ArrayList<Point2D>();
   private final ArrayList<MinMaxPointHolder> pointsToInterpolate = new ArrayList<MinMaxPointHolder>();
   
   public void setStartPoint(Point2D startPoint)
   {
      this.startPoint.set(startPoint);
   }

   public void setEndPoint(Point2D endPoint)
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
   public void stretchString(List<Point2D> waypointListToPack)
   {
      waypointListToPack.clear();
      pointsToInterpolate.clear();
      recycleableWayPoints.clear();
      
      findWaypoints(waypoints);

      Point2D leftPoint = startPoint;
      waypointListToPack.add(startPoint);

      int minMaxPointsIndex = 0;
      int waypointsIndex = 0;

      boolean done = false;

      while (!done)
      {
         MinMaxPointHolder minMaxPoint = getMinMaxPoint(minMaxPointsIndex);
         Point2D waypoint = getWaypoint(waypoints, waypointsIndex);

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
               MinMaxPointHolder pointToInterpolate = pointsToInterpolate.get(i);
               Point2D wayPoint = recycleableWayPoints.add();
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
         MinMaxPointHolder pointToInterpolate = pointsToInterpolate.get(i);
         Point2D wayPoint = recycleableWayPoints.add();
         interpolate(leftPoint, endPoint, pointToInterpolate.getMinPoint().getX(), wayPoint);
         waypointListToPack.add(wayPoint);
      }
      
      waypointListToPack.add(endPoint);
   }

   private Point2D getWaypoint(ArrayList<Point2D> waypoints, int waypointsIndex)
   {
      if (waypointsIndex < 0) return null;
      if (waypointsIndex >= waypoints.size()) return null;
      
      Point2D waypoint = waypoints.get(waypointsIndex);
      return waypoint;
   }

   private MinMaxPointHolder getMinMaxPoint(int minMaxPointsIndex)
   {
      if (minMaxPointsIndex < 0) return null;
      if (minMaxPointsIndex >= minMaxPoints.size()) return null;
      
      MinMaxPointHolder minMaxPoint = minMaxPoints.get(minMaxPointsIndex);
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
         Point2D waypoint = minMaxPoints.get(worstMinViolatorIndex).getMinPoint();
         insertWaypointBeforeEndPoint(waypointsToPack, endPoint, waypoint);
         
         findWaypoints(waypointsToPack, startPoint, waypoint, startIndex, worstMinViolatorIndex-1);
         findWaypoints(waypointsToPack, waypoint, endPoint, worstMinViolatorIndex + 1, endIndex);
      }

      else
      {
         int worstMaxViolatorIndex = findWorstMaxViolatorIndex(startPoint, endPoint, startIndex, endIndex);

         if (worstMaxViolatorIndex != -1)
         {
            Point2D waypoint = minMaxPoints.get(worstMaxViolatorIndex).getMaxPoint();
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


   private void interpolate(Point2D startPoint, Point2D endPoint, double x, Point2D midPointToPack)
   {
      double startX = startPoint.getX();
      double endX = endPoint.getX();

      double startY = startPoint.getY();
      double endY = endPoint.getY();

      double percentX = (x - startX) / (endX - startX);
      double y = startY + percentX * (endY - startY);
      
      midPointToPack.set(x, y);
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
      while ((minMaxPoints.size() > i) && (x > minMaxPoints.get(i).getMinPoint().getX()))
      {
         i++;
      }
      
      MinMaxPointHolder pointHolder = recycleablePointHolders.add();
      pointHolder.setMaxPoint(maxPoint);
      pointHolder.setMinPoint(minPoint);
      this.minMaxPoints.add(i, pointHolder);
   }


   public Point2D findWorstMinViolator(Point2D startPoint, Point2D endPoint)
   {
      int worstViolatorIndex = findWorstMinViolatorIndex(startPoint, endPoint, 0, minMaxPoints.size()-1);
      if (worstViolatorIndex == -1)
         return null;

      MinMaxPointHolder minMaxPoint = getMinMaxPoint(worstViolatorIndex);

      return minMaxPoint.getMinPoint();
   }

   private int findWorstMinViolatorIndex(Point2D startPoint, Point2D endPoint, int startIndex, int endIndex)
   {
      tempLine.set(startPoint, endPoint);

      int returnIndex = -1;
      double worstViolation = Double.NEGATIVE_INFINITY;

      for (int i = startIndex; i <= endIndex; i++)
      {
         MinMaxPointHolder minMaxPoint = getMinMaxPoint(i);
         Point2D minPoint = minMaxPoint.getMinPoint();
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

      MinMaxPointHolder minMaxPoint = getMinMaxPoint(worstViolatorIndex);

      return minMaxPoint.getMaxPoint();
   }

   private int findWorstMaxViolatorIndex(Point2D startPoint, Point2D endPoint, int startIndex, int endIndex)
   {
      tempLine.set(startPoint, endPoint);

      int returnIndex = -1;
      double worstViolation = Double.NEGATIVE_INFINITY;

      for (int i = startIndex; i <= endIndex; i++)
      {
         MinMaxPointHolder minMaxPoint = getMinMaxPoint(i);

         Point2D maxPoint = minMaxPoint.getMaxPoint();
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

   public MinMaxPointHolder findMinMaxPoints(double x)
   {
      for (int i = 0; i < minMaxPoints.size(); i++)
      {
         MinMaxPointHolder minMaxPoint = minMaxPoints.get(i);
         if (Math.abs(x - minMaxPoint.getMinPoint().getX()) < 1e-7) return minMaxPoint;
      }
      
      return null;
   }

   public void reset()
   {
      minMaxPoints.clear();
      recycleablePointHolders.clear();
   }
}
