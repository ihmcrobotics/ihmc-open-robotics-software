package us.ihmc.plotting;

import java.io.Serializable;

public class Coordinate implements Serializable
{
   private static final long serialVersionUID = 6493703987669931932L;
   /*
    * Constants
    */
   public final static int MILLIMETER = 0;
   public final static int CENTIMETER = 1;
   public final static int METER = 2;
   public final static int KILOMETER = 3;
   public final static int INCH = 4;
   public final static int FOOT = 5;
   public final static int MILE = 6;
   public final static int PIXEL = 7;

   /*
    *  Instance Attributes
    */
   private double x;
   private double y;
   private double z;
   private int unit;

   /*
    * Constructors
    */
   public Coordinate(double x, double y, int unit)
   {
      this.x = x;
      this.y = y;
      this.z = 0.0;
      this.unit = unit;
   }

   public Coordinate(double x, double y, double z, int unit)
   {
      this.x = x;
      this.y = y;
      this.z = z;
      this.unit = unit;
   }


   public Coordinate()
   {
      // needed for GPS
   }

   /*
    * Instance Methods
    */
   public double getX()
   {
      return x;
   }

   public double getY()
   {
      return y;
   }

   public double getZ()
   {
      return z;
   }

   public void setX(double x)
   {
      this.x = x;
   }

   public void setY(double y)
   {
      this.y = y;
   }

   public void setZ(double z)
   {
      this.z = z;
   }

   public int getUnit()
   {
      return unit;
   }

   public boolean equals(Coordinate coordinate)
   {
      if (coordinate == null)
         return false;
      if ((x == coordinate.getX()) && (y == coordinate.getY()))
         return true;

      return false;
   }

   public String toString()
   {
      return "(" + getX() + ", " + getY() + ", " + getZ() + ")";
   }
}
