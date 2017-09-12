package us.ihmc.robotics.quadTree;

import java.util.ArrayList;
import java.util.Collection;

import us.ihmc.euclid.tuple3D.Point3D;

public class QuadTreeForGroundLeaf
{
   private final ArrayList<QuadTreeForGroundPoint> points = new ArrayList<QuadTreeForGroundPoint>();
   private Point3D averagePoint = null; // To mark as dirty set to NaN.
   
   private final QuadTreeForGroundNode node;
   private final QuadTreeForGroundPointLimiter pointLimiter;

   public QuadTreeForGroundLeaf(QuadTreeForGroundNode node, QuadTreeForGroundPointLimiter pointLimiter)
   {
      this.node = node;
      this.pointLimiter = pointLimiter;
   }

   public Point3D getAveragePoint()
   {
      if (averagePoint == null)
      {
         averagePoint = new Point3D(Double.NaN, Double.NaN, Double.NaN);
      }

      if (Double.isNaN(averagePoint.getX()))
      {
         averagePoint.set(0.0, 0.0, 0.0);
         int numberOfPoints = points.size();
         for (int i = 0; i < numberOfPoints; i++)
         {
            averagePoint.add(points.get(i));
         }

         averagePoint.scale(1.0 / ((double) numberOfPoints));
      }

      return averagePoint;
   }

   public void clear()
   {
      if(pointLimiter != null)
      {
         for(int i = 0; i < points.size(); i++)
         {
            pointLimiter.remove(points.get(i));
         }
      }
      points.clear();
      setAveragePointDirty();
   }

   public ArrayList<QuadTreeForGroundPoint> getPoints()
   {
      return points;
   }

   public int getNumberOfPoints()
   {
      return points.size();
   }

   public void addPoint(QuadTreeForGroundPoint point)
   {
      this.points.add(point);
      point.setParent(this);
      if (pointLimiter != null)
      {
         pointLimiter.add(point);
      }
      
      
      setAveragePointDirty();
   }

   public void replaceLeastRecentPoint(QuadTreeForGroundPoint point)
   {
      setAveragePointDirty();
      QuadTreeForGroundPoint removed = this.points.remove(0);
      this.points.add(point);
      point.setParent(this);

      if (pointLimiter != null)
      {
         if (removed != null)
         {
            pointLimiter.remove(removed);
         }
         pointLimiter.add(point);
      }
   }

   private void setAveragePointDirty()
   {
      if (averagePoint != null)
         this.averagePoint.set(Double.NaN, Double.NaN, Double.NaN);
   }

   public void getClosestPointAndDistanceUsingAverage(double x, double y, PointAndDistance closestPointAndDistance)
   {
      Point3D averagePoint = getAveragePoint();

      double distanceSquared = distanceXYSquared(x, y, averagePoint);
      double bestDistanceSquared = closestPointAndDistance.getDistance() * closestPointAndDistance.getDistance();

      if (distanceSquared < bestDistanceSquared)
      {
         closestPointAndDistance.setPoint(averagePoint);
         closestPointAndDistance.setDistance(Math.sqrt(distanceSquared));
      }
   }

   public void getClosestPointAndDistanceUsingAverageHeight(double x, double y, PointAndDistance closestPointAndDistance)
   {
      double bestDistanceSquared = closestPointAndDistance.getDistance() * closestPointAndDistance.getDistance();

      Point3D betterPoint = null;
      for (Point3D point : points)
      {
         double distanceSquared = distanceXYSquared(x, y, point);
         if (distanceSquared < bestDistanceSquared)
         {
            betterPoint = point;
            bestDistanceSquared = distanceSquared;
         }
      }

      if (betterPoint != null)
      {
         Point3D averagePoint = getAveragePoint();
         closestPointAndDistance.setPoint(betterPoint);
         closestPointAndDistance.setPointZ(averagePoint.getZ()); // Use the average for z to do some filtering...

         closestPointAndDistance.setDistance(Math.sqrt(bestDistanceSquared));
      }
   }

   private double distanceXYSquared(double x, double y, Point3D point)
   {
      if (point == null)
         return Double.POSITIVE_INFINITY;

      return ((x - point.getX()) * (x - point.getX()) + (y - point.getY()) * (y - point.getY()));
   }

   public void getAllPoints(Collection<Point3D> pointsToPack)
   {
      pointsToPack.addAll(points);
   }

   public void getAllPointsWithinBounds(Box bounds, ArrayList<Point3D> pointsToPack)
   {
      for (Point3D point : points)
      {
         if (bounds.containsOrEquals(point.getX(), point.getY()))
         {
            pointsToPack.add(point);
         }
      }
   }

   public void getAllPointsWithinDistance(double x, double y, double maxDistance, ArrayList<Point3D> pointsWithinDistanceToPack)
   {
      if (maxDistance < 0.0)
         return;

      double maxDistanceSquared = maxDistance * maxDistance;

      for (Point3D point : points)
      {
         double distanceSquared = ((point.getX() - x) * (point.getX() - x)) + ((point.getY() - y) * (point.getY() - y));

         if (distanceSquared < maxDistanceSquared)
         {
            pointsWithinDistanceToPack.add(point);
         }
      }
   }

   public void checkRepInvarients(Box bounds)
   {
      if ((points == null) || (points.isEmpty()))
      {
         throw new RuntimeException("All leaves should have at least one point");
      }

      if ((points != null) && (!points.isEmpty()))
      {
         for (Point3D point : points)
         {
            if (!bounds.containsOrEquals(point.getX(), point.getY()))
               throw new RuntimeException();
         }
      }

      if ((averagePoint != null) && (Double.isNaN(averagePoint.getX())))
      {
         Point3D averageCheck = new Point3D(averagePoint);
         averagePoint.set(Double.NaN, Double.NaN, Double.NaN);
         if (averageCheck.distance(this.getAveragePoint()) > 1e-7)
            throw new RuntimeException();
      }
   }

   @Override
   public String toString()
   {
      return "QuadLeaf. Number of points = " + points.size() + ". " + points.toString();
   }

   // This is a callback from the decay, no need to call decay.remove again
   public void removePoint(QuadTreeForGroundPoint quadTreeForGroundPoint)
   {
      points.remove(quadTreeForGroundPoint);
      if(points.size() == 0)
      {
         node.merge();
      }
   }
}
