package us.ihmc.robotics.geometry;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class RevisedStringStretcher2d
{
   private final Point2D startPoint = new Point2D();
   private final Point2D endPoint = new Point2D();

   private final RecyclingArrayList<RevisedMinMaxPointHolder> minMaxPool = new RecyclingArrayList<>(20, RevisedMinMaxPointHolder.class);
   private final RecyclingArrayList<Point2D> waypointPool = new RecyclingArrayList<>(20, Point2D.class);

   private final List<RevisedMinMaxPointHolder> minMaxPoints = new ArrayList<>();

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
      waypointPool.clear();

      waypointListToPack.add(startPoint);

      findWaypoints(waypointListToPack);

      waypointListToPack.add(endPoint);
   }

   public void findWaypoints(List<Point2DBasics> waypointsToPack)
   {
      Point2DBasics leftPoint = startPoint;

      int minMaxIndex = 0;
      while (minMaxIndex < minMaxPoints.size())
      {
         RevisedMinMaxPointHolder minMaxPointHolder = minMaxPoints.get(minMaxIndex);
         Point2DBasics waypoint = waypointPool.add();

         interpolate(leftPoint, endPoint, minMaxPointHolder.getX(), waypoint);
         waypoint.setY(MathTools.clamp(waypoint.getY(), minMaxPointHolder.getMinY(), minMaxPointHolder.getMaxY()));
         leftPoint = waypoint;

         waypointsToPack.add(waypoint);

         minMaxIndex++;
      }
   }

   private static void interpolate(Point2DReadOnly startPoint, Point2DReadOnly endPoint, double x, Point2DBasics midPointToPack)
   {
      double startX = startPoint.getX();
      double endX = endPoint.getX();

      double percentX = (x - startX) / (endX - startX);

      midPointToPack.interpolate(startPoint, endPoint, percentX);
   }


   public void addMinMaxPoints(double pointX, double minPointY, double maxPointY)
   {
      double x = pointX;
      if (minPointY > maxPointY)
         throw new RuntimeException("Min Y " + minPointY + " is greater than Max Y " + maxPointY + ".");

      if (x < startPoint.getX())
         throw new RuntimeException("Min X " + x + "  is less than the start point " + startPoint.getX() + ".");
      if (x > endPoint.getX())
         throw new RuntimeException("Min X " + x + "  is greater than the end point " + endPoint.getX() + ".");

      int i = 0;
      while ((minMaxPoints.size() > i) && (x > minMaxPoints.get(i).getX()))
      {
         i++;
      }

      RevisedMinMaxPointHolder pointHolder = minMaxPool.add();
      pointHolder.set(pointX, minPointY, maxPointY);
      this.minMaxPoints.add(i, pointHolder);
   }



   public RevisedMinMaxPointHolder findMinMaxPoints(double x)
   {
      for (int i = 0; i < minMaxPoints.size(); i++)
      {
         RevisedMinMaxPointHolder minMaxPoint = minMaxPoints.get(i);
         if (MathTools.epsilonEquals(x, minMaxPoint.getX(), 1e-7))
            return minMaxPoint;
      }

      return null;
   }

   public void reset()
   {
      minMaxPoints.clear();
      minMaxPool.clear();
   }
}
