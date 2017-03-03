package us.ihmc.robotics.geometry;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.MathTools;

public class LineSegment1d
{
   private double endpoint1 = Double.NaN;
   private double endpoint2 = Double.NaN;
   private boolean positiveDirection;

   public LineSegment1d()
   {
   }

   public LineSegment1d(double firstEndpoint, double secondEndpoint)
   {
      set(firstEndpoint, secondEndpoint);
   }

   public LineSegment1d(double[] endpoints)
   {
      set(endpoints);
   }

   public boolean computeOverlap(LineSegment1d other, LineSegment1d intersectionToPack)
   {
      if (!isOverlappingInclusive(other))
         return false;

      double intersectionMin = Math.max(getMinPoint(), other.getMinPoint());
      double intersectionMax = Math.min(getMaxPoint(), other.getMaxPoint());
      intersectionToPack.set(intersectionMin, intersectionMax);
      return true;
   }

   public LineSegment1d computeOverlap(LineSegment1d other)
   {
      if (!isOverlappingInclusive(other))
         return null;

      double intersectionMin = Math.max(getMinPoint(), other.getMinPoint());
      double intersectionMax = Math.min(getMaxPoint(), other.getMaxPoint());
      return new LineSegment1d(intersectionMin, intersectionMax);
   }

   public boolean isOverlappingInclusive(LineSegment1d other)
   {
      return isBetweenEndpointsInclusive(other.endpoint1) || isBetweenEndpointsInclusive(other.endpoint2) || other.isBetweenEndpointsInclusive(endpoint1) || other.isBetweenEndpointsInclusive(endpoint2);
   }

   public boolean isOverlappingExclusive(LineSegment1d other)
   {
      return isBetweenEndpointsExclusive(other.endpoint1) || isBetweenEndpointsExclusive(other.endpoint2) || other.isBetweenEndpointsExclusive(endpoint1) || other.isBetweenEndpointsExclusive(endpoint2);
   }

   public boolean isBetweenEndpointsInclusive(LineSegment1d other)
   {
      return isBetweenEndpointsInclusive(other.endpoint1) && isBetweenEndpointsInclusive(other.endpoint2);
   }

   public boolean isBetweenEndpointsExclusive(LineSegment1d other)
   {
      return isBetweenEndpointsExclusive(other.endpoint1) && isBetweenEndpointsExclusive(other.endpoint2);
   }

   public boolean isBetweenEndpointsInclusive(double point)
   {
      return MathTools.intervalContains(point, getMinPoint(), getMaxPoint());
   }

   public boolean isBetweenEndpointsExclusive(double point)
   {
      return MathTools.intervalContains(point, getMinPoint(), getMaxPoint(), false, false);
   }

   public boolean isBetweenEndpoints(double point, double epsilon)
   {
      if (endpoint1 == endpoint2)
      {
         if (epsilon > 0.0)
            return false;
         else
            return point == endpoint1;
      }
      else if (epsilon > length())
      {
         return false;
      }

      double min = getMinPoint();
      double max = getMaxPoint();

      if (point < min + epsilon)
         return false;
      if (point > max - epsilon)
         return false;
      return true;
   }

   public double distance(double point)
   {
      if (isBetweenEndpointsInclusive(point))
         return -Math.min(point - getMinPoint(), getMaxPoint() - point);
      else if (point < getMinPoint())
         return getMinPoint() - point;
      else
         return point - getMaxPoint();
   }

   public void extendSegmentToPoint(double point)
   {
      if (isBetweenEndpointsInclusive(point))
         return;
      if (point < getMinPoint())
         setMinPoint(point);
      else
         setMaxPoint(point);
   }

   public boolean isBefore(double point)
   {
      if (positiveDirection)
         return point < endpoint1;
      else
         return point > endpoint1;
   }

   public boolean isAfter(double point)
   {
      if (positiveDirection)
         return point > endpoint2;
      else
         return point < endpoint2;
   }

   public void set(double firstEndpoint, double secondEndpoint)
   {
      this.endpoint1 = firstEndpoint;
      this.endpoint2 = secondEndpoint;
      updateDirection();
   }

   public void set(double[] endpoints)
   {
      this.endpoint1 = endpoints[0];
      this.endpoint2 = endpoints[1];
      updateDirection();
   }

   public void setFirstEndpoint(double firstEndpoint)
   {
      this.endpoint1 = firstEndpoint;
      updateDirection();
   }

   public void setSecondEndpoint(double secondEndpoint)
   {
      this.endpoint2 = secondEndpoint;
      updateDirection();
   }

   public void setMinPoint(double newMinPoint)
   {
      if (newMinPoint >= getMaxPoint())
         throw new RuntimeException("Unexpected newMinPoint: " + newMinPoint + ", expected it to be less than the current max point: " + getMaxPoint());
      if (positiveDirection)
         endpoint1 = newMinPoint;
      else
         endpoint2 = newMinPoint;
      updateDirection();
   }

   public void setMaxPoint(double newMaxPoint)
   {
      if (newMaxPoint <= getMinPoint())
         throw new RuntimeException("Unexpected newMaxPoint: " + newMaxPoint + ", expected it to be greater than the current min point: " + getMinPoint());
      if (positiveDirection)
         endpoint2 = newMaxPoint;
      else
         endpoint1 = newMaxPoint;
      updateDirection();
   }

   public double getFirstEndpoint()
   {
      return endpoint1;
   }

   public double getSecondEndpoint()
   {
      return endpoint2;
   }

   public double getMaxPoint()
   {
      return positiveDirection ? endpoint2 : endpoint1;
   }

   public double getMinPoint()
   {
      return positiveDirection ? endpoint1 : endpoint2;
   }

   public double getMidPoint()
   {
      return 0.5 * (endpoint1 + endpoint2);
   }

   public double length()
   {
      return (positiveDirection ? 1.0 : -1.0) * (endpoint2 - endpoint1);
   }

   /**
    * Compute the 3D equivalent of this line segment.
    * The 3D equivalent of each end point is computed as follows:
    * {@code endPoint3d = endPoint1d * lineDirection3d + lineStart3d}. 
    * @param line3d the 3D line used as reference to compute the 3D line segment.
    * @return the 3D equivalent of this line segment.
    */
   public LineSegment3d toLineSegment3d(Line3d line3d)
   {
      return toLineSegment3d(line3d.getPoint(), line3d.getNormalizedVector());
   }

   /**
    * Compute the 3D equivalent of this line segment.
    * The 3D equivalent of each end point is computed as follows:
    * {@code endPoint3d = endPoint1d * direction3d + zero3d}. 
    * @param zero3d position of the 3D equivalent of an endpoint equal to zero.
    * @param direction3d direction toward greater values of {@code endPoint1d}.
    * @return the 3D equivalent of this line segment.
    */
   public LineSegment3d toLineSegment3d(Point3DReadOnly zero3d, Vector3DReadOnly direction3d)
   {
      LineSegment3d lineSegment3d = new LineSegment3d();
      lineSegment3d.getFirstEndpoint().scaleAdd(endpoint1, direction3d, zero3d);
      lineSegment3d.getSecondEndpoint().scaleAdd(endpoint2, direction3d, zero3d);
      return lineSegment3d;
   }

   /**
    * Compute the 2D equivalent of this line segment.
    * The 2D equivalent of each end point is computed as follows:
    * {@code endPoint2d = endPoint1d * direction2d + zero2d}. 
    * @param zero2d position of the 2D equivalent of an endpoint equal to zero.
    * @param direction2d direction toward greater values of {@code endPoint1d}.
    * @return the 2D equivalent of this line segment.
    */
   public LineSegment2d toLineSegment2d(Point2DReadOnly zero2d, Vector2DReadOnly direction2d)
   {
      LineSegment2d lineSegment2d = new LineSegment2d();
      lineSegment2d.getFirstEndpoint().scaleAdd(endpoint1, direction2d, zero2d);
      lineSegment2d.getSecondEndpoint().scaleAdd(endpoint2, direction2d, zero2d);
      return lineSegment2d;
   }

   @Override
   public String toString()
   {
      return "(" + endpoint1 + ")-(" + endpoint2 + ")";
   }

   private void updateDirection()
   {
      positiveDirection = endpoint1 <= endpoint2;
   }
}
