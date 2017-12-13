package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class ConnectionPoint3D implements Point3DBasics
{
   public static final double PRECISION = 0.001;
   public static final double INV_PRECISION = 1.0 / PRECISION;

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
      return round(x);
   }

   public double getRoundedY()
   {
      return round(y);
   }

   public double getRoundedZ()
   {
      return round(z);
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
         return epsilonEquals((Tuple3DReadOnly) obj, PRECISION);
      }
      catch (ClassCastException | NullPointerException e)
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "ConnectionPoint3D: " + EuclidCoreIOTools.getTuple3DString(this);
   }

   private static double round(double value)
   {
      return ((int) value * INV_PRECISION) * PRECISION;
   }
}
