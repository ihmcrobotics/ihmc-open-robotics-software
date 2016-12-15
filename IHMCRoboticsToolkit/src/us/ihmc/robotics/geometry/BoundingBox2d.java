package us.ihmc.robotics.geometry;

import javax.vecmath.Point2d;

public class BoundingBox2d
{
   protected final Point2d minPoint;
   protected final Point2d maxPoint;

   public BoundingBox2d(Point2d min, Point2d max)
   {
      this.minPoint = new Point2d(min);
      this.maxPoint = new Point2d(max);
      checkPoint();
   }

   public BoundingBox2d(double[] min, double[] max)
   {
      this.minPoint = new Point2d(min);
      this.maxPoint = new Point2d(max);
      checkPoint();
   }

   public BoundingBox2d(double minX, double minY, double maxX, double maxY)
   {
      this.minPoint = new Point2d(minX, minY);
      this.maxPoint = new Point2d(maxX, maxY);
      checkPoint();
   }

   public void checkPoint()
   {
      if (minPoint.getX() > maxPoint.getX())
         throw new RuntimeException("minPoint.getX() > maxPoint.getX() " + minPoint.getX() + ">" + maxPoint.getX());
      if (minPoint.getY() > maxPoint.getY())
         throw new RuntimeException("minPoint.getY() > maxPoint.getY() " + minPoint.getY() + ">" + maxPoint.getY());
   }

   public BoundingBox2d(BoundingBox2d p)
   {
      this(p.minPoint, p.maxPoint);
   }

   public BoundingBox2d()
   {
      minPoint = new Point2d();
      maxPoint = new Point2d();
   }

   public void set(double minX, double minY, double maxX, double maxY)
   {
      this.minPoint.set(minX, minY);
      this.maxPoint.set(maxX, maxY);
      checkPoint();
   }

   public void set(BoundingBox2d boundingBox)
   {
      this.minPoint.set(boundingBox.minPoint);
      this.maxPoint.set(boundingBox.maxPoint);
      checkPoint();
   }

   public void getMinPoint(Point2d min)
   {
      min.set(this.minPoint);
   }

   public void getMaxPoint(Point2d max)
   {
      max.set(this.maxPoint);
   }

   public Point2d getMaxPoint()
   {
      return maxPoint;
   }

   public Point2d getMinPoint()
   {
      return minPoint;
   }

   public void getCenterPointCopy(Point2d center)
   {
      center.set((minPoint.getX() + maxPoint.getX()) / 2.0, (minPoint.getY() + maxPoint.getY()) / 2.0);
   }

   public double getDiagonalLengthSquared()
   {
      return minPoint.distanceSquared(maxPoint);
   }

   /**
    * returns true if this boxes minimum point y value >= referenceY
    */
   public boolean isBoxAtOrAbove(double referenceY)
   {
      return (minPoint.getY() >= referenceY);
   }

   /**
    * returns true if this boxes maximum point y value <= referenceY
    */
   public boolean isBoxAtOrBelow(double referenceY)
   {
      return (maxPoint.getY() <= referenceY);
   }

   /**
    * returns true if this boxes maximum point x value <= referenceX
    */
   public boolean isBoxAtOrLeftOf(double referenceX)
   {
      return (maxPoint.getX() <= referenceX);
   }

   /**
    * returns true if this boxes minimum point x value >= referenceX
    */
   public boolean isBoxAtOrRightOf(double referenceX)
   {
      return (minPoint.getX() >= referenceX);
   }

   // TODO isInside is not consistent with the other methods (> vs. >=)
   public boolean isInside(Point2d point2d)
   {
      if (point2d.getY() < minPoint.getY())
      {
         return false;
      }

      if (point2d.getY() > maxPoint.getY())
      {
         return false;
      }

      if (point2d.getX() < minPoint.getX())
      {
         return false;
      }

      if (point2d.getX() > maxPoint.getX())
      {
         return false;
      }

      return true;
   }

   public boolean intersects(BoundingBox2d r)
   {
      return ((maxPoint.getX() >= r.minPoint.getX()) && (r.maxPoint.getX() >= minPoint.getX()) && (maxPoint.getY() >= r.minPoint.getY()) && (r.maxPoint.getY() >= minPoint.getY()));
   }

   public void getPointGivenParameters(Point2d point, double xParameter, double yParameter)
   {
      point.setX(minPoint.getX() + xParameter * (maxPoint.getX() - minPoint.getX()));
      point.setY(minPoint.getY() + yParameter * (maxPoint.getY() - minPoint.getY()));
   }

   @Override
   public String toString()
   {
      return "BoundingBox2d{" + "minPoint=" + minPoint + " , maxPoint=" + maxPoint + '}';
   }

}
