package us.ihmc.robotics;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * {@code Axis} can be used to provide a simple and readable way to refer the three main axes of a
 * coordinate system.
 */
public enum Axis
{
   /** The x-axis is usually associated with the forward direction. */
   X(1.0, 0.0, 0.0),
   /**
    * The y-axis is usually associated with the direction pointing to the left of the forward
    * direction and is horizontal.
    */
   Y(0.0, 1.0, 0.0),
   /**
    * The z-axis is usually associated with the vertical direction, parallel to the gravity vector
    * but pointing the opposite direction.
    */
   Z(0.0, 0.0, 1.0);

   public static final Axis[] values = values();

   private final Vector3DReadOnly axisVector;

   Axis(double x, double y, double z)
   {
      this.axisVector = new ImmutableVector3D(x, y, z);
   }

   /**
    * Gets the immutable reference to the unit-vector 3D representing this axis.
    * 
    * @return the vector representation of this axis.
    */
   public Vector3DReadOnly getAxisVector()
   {
      return axisVector;
   }

   private static class ImmutableVector3D implements Vector3DReadOnly
   {
      private final double x;
      private final double y;
      private final double z;

      public ImmutableVector3D(double x, double y, double z)
      {
         this.x = x;
         this.y = y;
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
   }

   /**
    * Gets the value of the tuple for the given axis.
    *
    * @return double value of tuple for axis
    */
   public static double get(Tuple3DBasics tuple, Axis axis)
   {
      switch (axis)
      {
      case X :
         return tuple.getX();

      case Y :
         return tuple.getY();

      case Z :
         return tuple.getZ();

      default :
         throw new IndexOutOfBoundsException();
      }
   }

   /**
    * Sets the value of the given tuple for the given axis to the given value.
    */
   public static void set(Tuple3DBasics tuple, Axis axis, double value)
   {
      switch (axis)
      {
      case X :
         tuple.setX(value);

         break;

      case Y :
         tuple.setY(value);

         break;

      case Z :
         tuple.setZ(value);

         break;

      default :
         throw new IndexOutOfBoundsException();
      }
   }
}
