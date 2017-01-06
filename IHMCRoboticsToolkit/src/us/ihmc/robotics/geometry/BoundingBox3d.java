package us.ihmc.robotics.geometry;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple3d;

import us.ihmc.robotics.MathTools;

public class BoundingBox3d
{
   private static final double DEFAULT_EPSILON = 0.0;
   private double epsilon;

   private final Point3d minPoint = new Point3d();
   private final Point3d maxPoint = new Point3d();

   public static BoundingBox3d createUsingCenterAndPlusMinusVector(Point3d center, Tuple3d plusMinusVector)
   {
      return createUsingCenterAndPlusMinusVector(center, plusMinusVector, DEFAULT_EPSILON);
   }

   public static BoundingBox3d createUsingCenterAndPlusMinusVector(Point3d center, Tuple3d plusMinusVector, double epsilon)
   {
      Point3d minimumPoint = new Point3d(center);
      Point3d maximumPoint = new Point3d(center);

      minimumPoint.sub(plusMinusVector);
      maximumPoint.add(plusMinusVector);

      return new BoundingBox3d(minimumPoint, maximumPoint, epsilon);
   }

   public static BoundingBox3d union(BoundingBox3d boundingBoxOne, BoundingBox3d boundingBoxTwo)
   {
      double minX = Math.min(boundingBoxOne.minPoint.getX() - boundingBoxOne.epsilon, boundingBoxTwo.minPoint.getX() - boundingBoxTwo.epsilon);
      double minY = Math.min(boundingBoxOne.minPoint.getY() - boundingBoxOne.epsilon, boundingBoxTwo.minPoint.getY() - boundingBoxTwo.epsilon);
      double minZ = Math.min(boundingBoxOne.minPoint.getZ() - boundingBoxOne.epsilon, boundingBoxTwo.minPoint.getZ() - boundingBoxTwo.epsilon);

      double maxX = Math.max(boundingBoxOne.maxPoint.getX() + boundingBoxOne.epsilon, boundingBoxTwo.maxPoint.getX() + boundingBoxTwo.epsilon);
      double maxY = Math.max(boundingBoxOne.maxPoint.getY() + boundingBoxOne.epsilon, boundingBoxTwo.maxPoint.getY() + boundingBoxTwo.epsilon);
      double maxZ = Math.max(boundingBoxOne.maxPoint.getZ() + boundingBoxOne.epsilon, boundingBoxTwo.maxPoint.getZ() + boundingBoxTwo.epsilon);

      Point3d unionMin = new Point3d(minX, minY, minZ);
      Point3d unionMax = new Point3d(maxX, maxY, maxZ);

      return new BoundingBox3d(unionMin, unionMax, 0.0);
   }

   public BoundingBox3d(Point3d min, Point3d max, double epsilon)
   {
      this.epsilon = epsilon;
      set(min, max);
   }

   public BoundingBox3d(Point3d min, Point3d max)
   {
      this(min, max, DEFAULT_EPSILON);
   }

   public BoundingBox3d(double xMin, double yMin, double zMin, double xMax, double yMax, double zMax, double epsilon)
   {
      this.epsilon = epsilon;
      set(xMin, yMin, zMin, xMax, yMax, zMax);
   }

   public BoundingBox3d(double xMin, double yMin, double zMin, double xMax, double yMax, double zMax)
   {
      this(xMin, yMin, zMin, xMax, yMax, zMax, DEFAULT_EPSILON);
   }

   public BoundingBox3d(double[] min, double[] max, double epsilon)
   {
      this.epsilon = epsilon;
      set(min, max);
   }

   public BoundingBox3d(double[] min, double[] max)
   {
      this(min, max, DEFAULT_EPSILON);
   }

   public BoundingBox3d(BoundingBox3d boundingBox, double epsilon)
   {
      this(boundingBox.minPoint, boundingBox.maxPoint, epsilon);
   }

   public BoundingBox3d(BoundingBox3d boundingBox)
   {
      this(boundingBox, boundingBox.epsilon);
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

   public void set(BoundingBox3d otherBoundingBox3d, boolean shouldSetEpsilon)
   {
      this.set(otherBoundingBox3d.getXMin(), otherBoundingBox3d.getYMin(), otherBoundingBox3d
                  .getZMin(), otherBoundingBox3d.getXMax(),
            otherBoundingBox3d.getYMax(), otherBoundingBox3d.getZMax());

      if(shouldSetEpsilon)
      {
         this.epsilon = otherBoundingBox3d.epsilon;
      }
   }

   public void set(BoundingBox3d otherBoundingBox3d)
   {
      this.set(otherBoundingBox3d, true);
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

   public boolean isInside(Point3f point3f)
   {
      return isInside(point3f.getX(), point3f.getY(), point3f.getZ());
   }

   // TODO isInside is not consistent with the other methods (> vs. >=)
   public boolean isXYInside(double x, double y)
   {
      if (y < (minPoint.getY() - epsilon))
      {
         return false;
      }

      if (y > (maxPoint.getY() + epsilon))
      {
         return false;
      }

      if (x < (minPoint.getX() - epsilon))
      {
         return false;
      }

      if (x > (maxPoint.getX() + epsilon))
      {
         return false;
      }

      return true;
   }

   // TODO isInside is not consistent with the other methods (> vs. >=)
   public boolean isInside(double x, double y, double z)
   {
      if (!isXYInside(x, y))
      {
         return false;
      }

      if (z < (minPoint.getZ() - epsilon))
      {
         return false;
      }

      if (z > (maxPoint.getZ() + epsilon))
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

   public boolean intersectsInXYPlane(BoundingBox2d boundingBox)
   {
      return maxPoint.getX() >= boundingBox.minPoint.getX() && boundingBox.maxPoint.getX() >= minPoint.getX() && maxPoint.getY() >= boundingBox.minPoint.getY()
            && boundingBox.maxPoint.getY() >= minPoint.getY();
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

   public void setEpsilonToShrink(double epsilon)
   {
      MathTools.checkIfNegative(epsilon);
      this.epsilon = epsilon;
   }

   public void setEpsilonToGrow(double epsilon)
   {
      MathTools.checkIfPositive(epsilon);
      this.epsilon = epsilon;
   }

   public void updateToIncludePoint(Tuple3d point)
   {
      this.updateToIncludePoint(point.x, point.y, point.z);
   }

   public void updateToIncludePoint(double x, double y, double z)
   {
      if (Double.isNaN(this.minPoint.x) || (x < this.minPoint.x))
      {
         this.minPoint.x = x;
      }

      if (Double.isNaN(this.minPoint.y) || (y < this.minPoint.y))
      {
         this.minPoint.y = y;
      }

      if (Double.isNaN(this.minPoint.z) || (z < this.minPoint.z))
      {
         this.minPoint.z = z;
      }

      if (Double.isNaN(this.maxPoint.x) || (x > this.maxPoint.x))
      {
         this.maxPoint.x = x;
      }

      if (Double.isNaN(this.maxPoint.y) || (y > this.maxPoint.y))
      {
         this.maxPoint.y = y;
      }

      if (Double.isNaN(this.maxPoint.z) || (z > this.maxPoint.z))
      {
         this.maxPoint.z = z;
      }
   }

   public boolean containsNaN()
   {
      return Double.isNaN(minPoint.getX()) || Double.isNaN(maxPoint.getX()) || Double.isNaN(minPoint.getY()) || Double.isNaN(maxPoint.getY()) || Double
            .isNaN(minPoint.getZ()) || Double.isNaN(maxPoint.getZ());
   }

   @Override
   public String toString()
   {
      return "BoundingBox3d{" + "minPoint=" + minPoint + " , maxPoint=" + maxPoint + '}';
   }
}
