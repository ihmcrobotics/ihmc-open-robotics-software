package us.ihmc.robotics.geometry;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;

public class BoundingBox3d
{
   private final Point3d minPoint;
   private final Point3d maxPoint;

   public static BoundingBox3d createUsingCenterAndPlusMinusVector(Point3d center, Tuple3d plusMinusVector)
   {
      Point3d minimumPoint = new Point3d(center);
      Point3d maximumPoint = new Point3d(center);

      minimumPoint.sub(plusMinusVector);
      maximumPoint.add(plusMinusVector);

      return new BoundingBox3d(minimumPoint, maximumPoint);
   }
   
   public BoundingBox3d(Point3d min, Point3d max)
   {
      this.minPoint = new Point3d(min);
      this.maxPoint = new Point3d(max);

      verifyBounds();
   }

   public BoundingBox3d(double xMin, double yMin, double zMin, double xMax, double yMax, double zMax)
   {
      this.minPoint = new Point3d(xMin, yMin, zMin);
      this.maxPoint = new Point3d(xMax, yMax, zMax);

      verifyBounds();
   }

   public BoundingBox3d(double[] min, double[] max)
   {
      this.minPoint = new Point3d(min);
      this.maxPoint = new Point3d(max);

      verifyBounds();
   }

   public BoundingBox3d(BoundingBox3d boundingBox)
   {
      this(boundingBox.minPoint, boundingBox.maxPoint);
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

   public void getMinPoint(Point3d min)
   {
      min.set(this.minPoint);
   }

   public void getMaxPoint(Point3d max)
   {
      max.set(this.maxPoint);
   }

   public void getCenterPointCopy(Point3d center)
   {
      center.set((minPoint.getX() + maxPoint.getX()) / 2.0, (minPoint.getY() + maxPoint.getY()) / 2.0, (minPoint.getZ() + maxPoint.getZ()) / 2.0);
   }

   /**
    * returns true if this boxes minimum point z value >= referenceZ
    */
   public boolean isBoxAtOrAbove(double referenceZ)
   {
      return (minPoint.getZ() >= referenceZ);
   }

   /**
    * returns true if this boxes maximum point z value <= referenceZ
    */
   public boolean isBoxAtOrBelow(double referenceZ)
   {
      return (maxPoint.getZ() <= referenceZ);
   }

   /**
    * returns true if this boxes minimum point y value >= referenceY
    */
   public boolean isBoxAtOrLeftOf(double referenceY)
   {
      return (minPoint.getY() >= referenceY);
   }

   /**
    * returns true if this boxes maximum point y value <= referenceY
    */
   public boolean isBoxAtOrRightOf(double referenceY)
   {
      return (maxPoint.getY() <= referenceY);
   }

   /**
    * returns true if this boxes maximum point x value <= referenceX
    */
   public boolean isBoxAtOrBehind(double referenceX)
   {
      return (maxPoint.getX() <= referenceX);
   }

   /**
    * returns true if this boxes minimum point x value >= referenceX
    */
   public boolean isBoxAtOrInFrontOf(double referenceX)
   {
      return (minPoint.getX() >= referenceX);
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
      return ((maxPoint.x >= boundingBox.minPoint.x) && (boundingBox.maxPoint.x >= minPoint.x) && (maxPoint.y >= boundingBox.minPoint.y)
            && (boundingBox.maxPoint.y >= minPoint.y) && (maxPoint.z >= boundingBox.minPoint.z) && (boundingBox.maxPoint.z >= minPoint.z));
   }

   public boolean intersects(Point3d start, Point3d end)
   {
      double invXDir = 1 / (end.x - start.x);
      double invYDir = 1 / (end.y - start.y);
      double invZDir = 1 / (end.z - start.z);

      double tmin, tmax, tymin, tymax, tzmin, tzmax;

      if (invXDir > 0)
      {
         tmin = (minPoint.x - start.x) * invXDir;
         tmax = (maxPoint.x - start.x) * invXDir;
      }
      else
      {
         tmin = (maxPoint.x - start.x) * invXDir;
         tmax = (minPoint.x - start.x) * invXDir;
      }
      
      if (invYDir > 0)
      {
         tymin = (minPoint.y - start.y) * invYDir;
         tymax = (maxPoint.y - start.y) * invYDir;
      }
      else
      {
         tymin = (maxPoint.y - start.y) * invYDir;
         tymax = (minPoint.y - start.y) * invYDir;
      }
      
      // if regions do not overlap, return false
      if (tmin > tymax || tmax < tymin) {
         return false;
      }
      
      // update tmin
      if (tymin > tmin)
         tmin = tymin;
      
      if (tymax < tmax)
         tmax = tymax;
      
      if (invZDir > 0)
      {
         tzmin = (minPoint.z - start.z) * invZDir;
         tzmax = (maxPoint.z - start.z) * invZDir;
      }
      else
      {
         tzmin = (maxPoint.z - start.z) * invZDir;
         tzmax = (minPoint.z - start.z) * invZDir;
      }
      
      // if regions do not overlap, return false
      if (tmin > tzmax || tmax < tzmin) {
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
