package us.ihmc.pathPlanning.visibilityGraphs;

import org.apache.commons.math3.util.Precision;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class ConnectionPoint3D implements Point3DBasics
{
   private static final double EPSILON = 1.0e-12;
   private static final int SCALE = 3;
   public static final double PRECISION = Math.pow(10.0, -SCALE); // Do not change PRECISION, change SCALE if needed.

   private double x, y, z;

   /**
    * Creates a new connection point and initializes it coordinates to zero.
    */
   public ConnectionPoint3D()
   {
      setToZero();
   }

   /**
    * Creates a new connection point and initializes it with the given coordinates.
    *
    * @param x the x-coordinate.
    * @param y the y-coordinate.
    * @param z the z-coordinate.
    */
   public ConnectionPoint3D(double x, double y, double z)
   {
      set(x, y, z);
   }

   /**
    * Creates a new connection point and initializes its component {@code x}, {@code y}, {@code z}
    * in order from the given array.
    *
    * @param pointArray the array containing this point's coordinates. Not modified.
    */
   public ConnectionPoint3D(double[] pointArray)
   {
      set(pointArray);
   }

   /**
    * Creates a new connection point and initializes its x and y components to
    * {@code tuple2DReadOnly}.
    *
    * @param tuple2DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public ConnectionPoint3D(Tuple2DReadOnly tuple2DReadOnly)
   {
      set(tuple2DReadOnly);
   }

   /**
    * Creates a new connection point and initializes it to {@code other}.
    *
    * @param other the tuple to copy the coordinates from. Not modified.
    */
   public ConnectionPoint3D(Tuple3DReadOnly other)
   {
      set(other);
   }

   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   @Override
   public void setZ(double z)
   {
      this.z = z;
   }

   @Override
   public double getX()
   {
      return x;
   }

   @Override
   public double getY()
   {
      return y;
   }

   @Override
   public double getZ()
   {
      return z;
   }

   public double getRoundedX()
   {
      return Precision.round(x, SCALE);
   }

   public double getRoundedY()
   {
      return Precision.round(y, SCALE);
   }

   public double getRoundedZ()
   {
      return Precision.round(z, SCALE);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = 31L * bits + Double.doubleToLongBits(getRoundedX());
      bits = 31L * bits + Double.doubleToLongBits(getRoundedY());
      bits = 31L * bits + Double.doubleToLongBits(getRoundedZ());
      return (int) (bits ^ bits >> 32);
   }

   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((ConnectionPoint3D) obj);
      }
      catch (ClassCastException e)
      {
         try
         {
            return equals((Tuple3DReadOnly) obj);
         }
         catch (ClassCastException e1)
         {
            return false;
         }
      }
   }

   public boolean equals(ConnectionPoint3D other)
   {
      try
      {
         if (!MathTools.epsilonEquals(getRoundedX(), other.getRoundedX(), EPSILON))
            return false;
         if (!MathTools.epsilonEquals(getRoundedY(), other.getRoundedY(), EPSILON))
            return false;
         if (!MathTools.epsilonEquals(getRoundedZ(), other.getRoundedZ(), EPSILON))
            return false;
         return true;
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }

   @Override
   public boolean equals(Tuple3DReadOnly point3D)
   {
      try
      {
         if (!MathTools.epsilonEquals(getRoundedX(), Precision.round(point3D.getX(), SCALE), EPSILON))
            return false;
         if (!MathTools.epsilonEquals(getRoundedY(), Precision.round(point3D.getY(), SCALE), EPSILON))
            return false;
         if (!MathTools.epsilonEquals(getRoundedZ(), Precision.round(point3D.getZ(), SCALE), EPSILON))
            return false;
         return true;
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "ConnectionPoint3D: " + EuclidCoreIOTools.getTuple3DString(this);
   }
}
