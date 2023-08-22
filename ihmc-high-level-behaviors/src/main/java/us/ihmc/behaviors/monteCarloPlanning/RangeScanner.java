package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.ArrayList;

public class RangeScanner
{
   private int numPoints;
   private int maxRange;
   private int maxRangeSquared;

   public RangeScanner(int numPoints, int maxRange)
   {
      this.numPoints = numPoints;
      this.maxRange = maxRange;
      this.maxRangeSquared = maxRange * maxRange;
   }

   public ArrayList<Point2DReadOnly> scan(Point2DReadOnly pos, MonteCarloPlanningWorld world)
   {
      ArrayList<Point2DReadOnly> points = new ArrayList<>();
      for (int i = 0; i < numPoints; i++)
      {
         float theta = i * 2 * (float) Math.PI / numPoints;
         Point2DReadOnly point = getScanPoint(pos, theta, world);
         points.add(point);
      }

      return points;
   }

   public Point2DReadOnly getScanPoint(Point2DReadOnly pos, float theta, MonteCarloPlanningWorld world)
   {
      // Get the end point of the ray
      Point2DReadOnly endPoint = new Point2D(pos.getX32() + maxRange * (float) Math.cos(theta), pos.getY32() + maxRange * (float) Math.sin(theta));

      // Get the intersection point with the obstacles
      Point2DReadOnly scanPoint = MonteCarloPlannerTools.findClosestOccupiedPoint(pos, endPoint, world.getGrid(), maxRange);

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

   public int getMaxRangeSquared()
   {
      return maxRangeSquared;
   }
}