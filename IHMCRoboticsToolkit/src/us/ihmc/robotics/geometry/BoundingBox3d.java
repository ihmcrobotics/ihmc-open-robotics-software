package us.ihmc.robotics.geometry;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;

public class BoundingBox3d
{
   private final Point3d minPoint = new Point3d();
   private final Point3d maxPoint = new Point3d();

   public static BoundingBox3d createUsingCenterAndPlusMinusVector(Point3d center, Tuple3d plusMinusVector)
   {
      Point3d minimumPoint = new Point3d(center);
      Point3d maximumPoint = new Point3d(center);

      minimumPoint.sub(plusMinusVector);
      maximumPoint.add(plusMinusVector);

      return new BoundingBox3d(minimumPoint, maximumPoint);
   }

   public static BoundingBox3d union(BoundingBox3d boundingBoxOne, BoundingBox3d boundingBoxTwo)
   {
      double minX = Math.min(boundingBoxOne.minPoint.getX(), boundingBoxTwo.minPoint.getX());
      double minY = Math.min(boundingBoxOne.minPoint.getY(), boundingBoxTwo.minPoint.getY());
      double minZ = Math.min(boundingBoxOne.minPoint.getZ(), boundingBoxTwo.minPoint.getZ());

      double maxX = Math.max(boundingBoxOne.maxPoint.getX(), boundingBoxTwo.maxPoint.getX());
      double maxY = Math.max(boundingBoxOne.maxPoint.getY(), boundingBoxTwo.maxPoint.getY());
      double maxZ = Math.max(boundingBoxOne.maxPoint.getZ(), boundingBoxTwo.maxPoint.getZ());

      Point3d unionMin = new Point3d(minX, minY, minZ);
      Point3d unionMax = new Point3d(maxX, maxY, maxZ);

      return new BoundingBox3d(unionMin, unionMax);
   }

   public BoundingBox3d(Point3d min, Point3d max)
   {
      set(min, max);
   }

   public BoundingBox3d(double xMin, double yMin, double zMin, double xMax, double yMax, double zMax)
   {
      set(xMin, yMin, zMin, xMax, yMax, zMax);
   }

   public BoundingBox3d(double[] min, double[] max)
   {
      set(min, max);
   }

   public BoundingBox3d(BoundingBox3d boundingBox)
   {
      this(boundingBox.minPoint, boundingBox.maxPoint);
   }

   public void set(Point3d min, Point3d max)
   {
      minPoint.set(min);
      maxPoint.set(max);

      verifyBounds();
   }

   public void set(double xMin, double yMin, double zMin, double xMax, double yMax, double zMax)
   {
      minPoint.set(xMin, yMin, zMin);
      maxPoint.set(xMax, yMax, zMax);

      verifyBounds();
   }

   public void set(BoundingBox3d otherBoundingBox3d)
   {
      this.set(otherBoundingBox3d.getXMin(), otherBoundingBox3d.getYMin(), otherBoundingBox3d
                  .getZMin(), otherBoundingBox3d.getXMax(),
            otherBoundingBox3d.getYMax(), otherBoundingBox3d.getZMax());
   }

   public void set(double[] min, double[] max)
   {
      minPoint.set(min);
      maxPoint.set(max);

      verifyBounds();
   }

   public void setMin(Point3d min)
   {
      minPoint.set(min);
      verifyBounds();
   }

   public void setMax(Point3d max)
   {
      maxPoint.set(max);
      verifyBounds();
   }

   private void verifyBounds()
   {
      if (minPoint.getX() > maxPoint.getX())
         throw new RuntimeException("minPoint.getX() > maxPoint.getX()");
      if (minPoint.getY() > maxPoint.getY())
         throw new RuntimeException("minPoint.getY() > maxPoint.getY()");
      if (minPoint.getZ() > maxPoint.getZ())
         throw new RuntimeException("minPoint.getZ() > maxPoint.getZ()");
   }

   public double getXMin()
   {
      return minPoint.getX();
   }

   public double getYMin()
   {
      return minPoint.getY();
   }

   public double getZMin()
   {
      return minPoint.getZ();
   }

   public double getXMax()
   {
      return maxPoint.getX();
   }

   public double getYMax()
   {
      return maxPoint.getY();
   }

   public double getZMax()
   {
      return maxPoint.getZ();
   }

   public void getMinPoint(Point3d minToPack)
   {
      minToPack.set(minPoint);
   }

   public void getMaxPoint(Point3d maxToPack)
   {
      maxToPack.set(maxPoint);
   }

   public void getMinPoint(double[] minToPack)
   {
      minPoint.get(minToPack);
   }

   public void getMaxPoint(double[] maxToPack)
   {
      maxPoint.get(maxToPack);
   }

   public void getCenterPoint(Point3d centerToPack)
   {
      centerToPack.interpolate(minPoint, maxPoint, 0.5);
   }

   /**
    * returns true if this boxes minimum point z value >= referenceZ
    */
   public boolean isBoxAtOrAbove(double referenceZ)
   {
      return minPoint.getZ() >= referenceZ;
   }

   /**
    * returns true if this boxes maximum point z value <= referenceZ
    */
   public boolean isBoxAtOrBelow(double referenceZ)
   {
      return maxPoint.getZ() <= referenceZ;
   }

   /**
    * returns true if this boxes minimum point y value >= referenceY
    */
   public boolean isBoxAtOrLeftOf(double referenceY)
   {
      return minPoint.getY() >= referenceY;
   }

   /**
    * returns true if this boxes maximum point y value <= referenceY
    */
   public boolean isBoxAtOrRightOf(double referenceY)
   {
      return maxPoint.getY() <= referenceY;
   }

   /**
    * returns true if this boxes maximum point x value <= referenceX
    */
   public boolean isBoxAtOrBehind(double referenceX)
   {
      return maxPoint.getX() <= referenceX;
   }

   /**
    * returns true if this boxes minimum point x value >= referenceX
    */
   public boolean isBoxAtOrInFrontOf(double referenceX)
   {
      return minPoint.getX() >= referenceX;
   }

   public boolean isInside(Point3d point3d)
   {
      return isInside(point3d.getX(), point3d.getY(), point3d.getZ());
   }

   // TODO isInside is not consistent with the other methods (> vs. >=)
   public boolean isXYInside(double x, double y)
   {
      if (y < minPoint.getY())
      {
         return false;
      }

      if (y > maxPoint.getY())
      {
         return false;
      }

      if (x < minPoint.getX())
      {
         return false;
      }

      if (x > maxPoint.getX())
      {
         return false;
      }

      return true;
   }

   // TODO isInside is not consistent with the other methods (> vs. >=)
   public boolean isInside(double x, double y, double z)
   {
      if (!isXYInside(x, y))
         return false;

      if (z < minPoint.getZ())
      {
         return false;
      }

      if (z > maxPoint.getZ())
      {
         return false;
      }

      return true;
   }

   public boolean intersects(BoundingBox3d boundingBox)
   {
      return maxPoint.getX() >= boundingBox.minPoint.getX() && boundingBox.maxPoint.getX() >= minPoint.getX() && maxPoint.getY() >= boundingBox.minPoint.getY()
            && boundingBox.maxPoint.getY() >= minPoint.getY() && maxPoint.getZ() >= boundingBox.minPoint.getZ()
            && boundingBox.maxPoint.getZ() >= minPoint.getZ();
   }

   public boolean intersects(Point3d start, Point3d end)
   {
      double invXDir = 1 / (end.getX() - start.getX());
      double invYDir = 1 / (end.getY() - start.getY());
      double invZDir = 1 / (end.getZ() - start.getZ());

      double tmin, tmax, tymin, tymax, tzmin, tzmax;

      if (invXDir > 0)
      {
         tmin = (minPoint.getX() - start.getX()) * invXDir;
         tmax = (maxPoint.getX() - start.getX()) * invXDir;
      }
      else
      {
         tmin = (maxPoint.getX() - start.getX()) * invXDir;
         tmax = (minPoint.getX() - start.getX()) * invXDir;
      }

      if (invYDir > 0)
      {
         tymin = (minPoint.getY() - start.getY()) * invYDir;
         tymax = (maxPoint.getY() - start.getY()) * invYDir;
      }
      else
      {
         tymin = (maxPoint.getY() - start.getY()) * invYDir;
         tymax = (minPoint.getY() - start.getY()) * invYDir;
      }

      // if regions do not overlap, return false
      if (tmin > tymax || tmax < tymin)
      {
         return false;
      }

      // update tmin
      if (tymin > tmin)
         tmin = tymin;

      if (tymax < tmax)
         tmax = tymax;

      if (invZDir > 0)
      {
         tzmin = (minPoint.getZ() - start.getZ()) * invZDir;
         tzmax = (maxPoint.getZ() - start.getZ()) * invZDir;
      }
      else
      {
         tzmin = (maxPoint.getZ() - start.getZ()) * invZDir;
         tzmax = (minPoint.getZ() - start.getZ()) * invZDir;
      }

      // if regions do not overlap, return false
      if (tmin > tzmax || tmax < tzmin)
      {
         return false;
      }

      // update tmin
      if (tzmin > tmin)
         tmin = tzmin;

      if (tzmax < tmax)
         tmax = tzmax;

      // return tmin/tmax within 0-1
      return tmin < 1 && tmax > 0;

   }

   @Override
   public String toString()
   {
      return "BoundingBox3d{" + "minPoint=" + minPoint + " , maxPoint=" + maxPoint + '}';
   }
}
