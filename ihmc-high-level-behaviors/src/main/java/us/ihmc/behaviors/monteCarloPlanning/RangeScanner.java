package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple4D.Vector4D32;

import java.util.ArrayList;

public class RangeScanner
{
   private final int numPoints = 24;
   private final int maxRange = 20;

   public ArrayList<Point2D> scan(Point2D pos, World world)
   {
      ArrayList<Point2D> points = new ArrayList<>();
      for (int i = 0; i < numPoints; i++)
      {
         float theta = i * 2 * (float) Math.PI / numPoints;
         Point2D point = getScanPoint(pos, theta, world);
         points.add(point);
      }

      return points;
   }

   public Point2D getScanPoint(Point2D pos, float theta, World world)
   {
      // Get the end point of the ray
      Point2D end_point = new Point2D(pos.getX32() + maxRange * (float) Math.cos(theta), pos.getY32() + maxRange * (float) Math.sin(theta));

      // Get the intersection point with the obstacles
      Point2D scanPoint = MonteCarloPlannerTools.findClosestIntersection(pos, end_point, world.getGrid());

      if (scanPoint.distance(pos) > maxRange)
      {
         scanPoint = end_point;
      }

      return scanPoint;
   }

   public int getNumPoints()
   {
      return numPoints;
   }

   public int getMaxRange()
   {
      return maxRange;
   }
}