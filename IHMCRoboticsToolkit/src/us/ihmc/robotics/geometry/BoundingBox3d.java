package us.ihmc.robotics.geometry;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.MathTools;

public class BoundingBox3d
{
   private static final double DEFAULT_EPSILON = 0.0;
   private double epsilon;

   private final Point3D minPoint = new Point3D();
   private final Point3D maxPoint = new Point3D();

   public static BoundingBox3d createUsingCenterAndPlusMinusVector(Point3DReadOnly center, Tuple3DReadOnly plusMinusVector)
   {
      return createUsingCenterAndPlusMinusVector(center, plusMinusVector, DEFAULT_EPSILON);
   }

   public static BoundingBox3d createUsingCenterAndPlusMinusVector(Point3DReadOnly center, Tuple3DReadOnly plusMinusVector, double epsilon)
   {
      Point3D minimumPoint = new Point3D(center);
      Point3D maximumPoint = new Point3D(center);

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

      Point3D unionMin = new Point3D(minX, minY, minZ);
      Point3D unionMax = new Point3D(maxX, maxY, maxZ);

      return new BoundingBox3d(unionMin, unionMax, 0.0);
   }

   public BoundingBox3d(Point3DReadOnly min, Point3DReadOnly max, double epsilon)
   {
      this.epsilon = epsilon;
      set(min, max);
   }

   public BoundingBox3d(Point3DReadOnly min, Point3DReadOnly max)
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

   public void set(Point3DReadOnly min, Point3DReadOnly max)
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

   public void setMin(Point3DReadOnly min)
   {
      minPoint.set(min);
      verifyBounds();
   }

   public void setMax(Point3DReadOnly max)
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

   public void getMinPoint(Point3DBasics minToPack)
   {
      minToPack.set(minPoint);
   }

   public void getMaxPoint(Point3DBasics maxToPack)
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

   public void getCenterPoint(Point3DBasics centerToPack)
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

   public boolean isInside(Point3D point3d)
   {
      return isInside(point3d.getX(), point3d.getY(), point3d.getZ());
   }

   public boolean isInside(Point3D32 point3f)
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

   public boolean intersects(Point3DReadOnly start, Point3DReadOnly end)
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
      MathTools.checkNegative(epsilon);
      this.epsilon = epsilon;
   }

   public void setEpsilonToGrow(double epsilon)
   {
      MathTools.checkPositive(epsilon);
      this.epsilon = epsilon;
   }

   public void updateToIncludePoint(Tuple3DReadOnly point)
   {
      this.updateToIncludePoint(point.getX(), point.getY(), point.getZ());
   }

   public void updateToIncludePoint(double x, double y, double z)
   {
      if (Double.isNaN(this.minPoint.getX()) || (x < this.minPoint.getX()))
      {
         this.minPoint.setX(x);
      }

      if (Double.isNaN(this.minPoint.getY()) || (y < this.minPoint.getY()))
      {
         this.minPoint.setY(y);
      }

      if (Double.isNaN(this.minPoint.getZ()) || (z < this.minPoint.getZ()))
      {
         this.minPoint.setZ(z);
      }

      if (Double.isNaN(this.maxPoint.getX()) || (x > this.maxPoint.getX()))
      {
         this.maxPoint.setX(x);
      }

      if (Double.isNaN(this.maxPoint.getY()) || (y > this.maxPoint.getY()))
      {
         this.maxPoint.setY(y);
      }

      if (Double.isNaN(this.maxPoint.getZ()) || (z > this.maxPoint.getZ()))
      {
         this.maxPoint.setZ(z);
      }
   }

   public boolean containsNaN()
   {
      return minPoint.containsNaN() || maxPoint.containsNaN();
   }

   @Override
   public String toString()
   {
      return "BoundingBox3d{" + "minPoint=" + minPoint + " , maxPoint=" + maxPoint + '}';
   }
}
