package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class ConnectionPoint3D implements Point3DReadOnly
{
   public static final double PRECISION     = 1.0e-4;
   public static final double INV_PRECISION = 1.0e+4;

   private final int regionId;
   private final double x, y, z;
   private final int hashCode;

   public ConnectionPoint3D(int regionId)
   {
      x = 0.0;
      y = 0.0;
      z = 0.0;
      this.regionId = regionId;
      hashCode = computeHashCode();
   }

   public ConnectionPoint3D(ConnectionPoint3D other)
   {
      this(other, other.regionId);
   }

   public ConnectionPoint3D(double x, double y, double z, int regionId)
   {
      this.x = x;
      this.y = y;
      this.z = z;
      this.regionId = regionId;
      hashCode = computeHashCode();
   }

   public ConnectionPoint3D(Tuple2DReadOnly tuple2DReadOnly, int regionId)
   {
      x = tuple2DReadOnly.getX();
      y = tuple2DReadOnly.getY();
      z = 0.0;
      this.regionId = regionId;
      hashCode = computeHashCode();
   }

   public ConnectionPoint3D(Tuple3DReadOnly other, int regionId)
   {
      x = other.getX();
      y = other.getY();
      z = other.getZ();
      this.regionId = regionId;
      hashCode = computeHashCode();
   }

   private int computeHashCode()
   {
      long bits = 1L;
      bits = 31L * bits + Double.doubleToLongBits(getRoundedX());
      bits = 31L * bits + Double.doubleToLongBits(getRoundedY());
      bits = 31L * bits + Double.doubleToLongBits(getRoundedZ());
      return (int) (bits ^ bits >> 32);
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

   public int getRegionId()
   {
      return regionId;
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
      return hashCode;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == null)
         return false;

      try
      {
         return epsilonEquals((Tuple3DReadOnly) obj, PRECISION);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   @Override
   public boolean equals(Tuple3DReadOnly other)
   {
      return epsilonEquals(other, PRECISION);
   }

   @Override
   public String toString()
   {
      return "ConnectionPoint3D: " + EuclidCoreIOTools.getTuple3DString(this);
   }

   static double round(double value)
   {
      return Math.round(value * INV_PRECISION) * PRECISION;
   }

   public ConnectionPoint3D applyTransform(Transform transform)
   {
      Point3D transformed = new Point3D(this);
      transformed.applyTransform(transform);
      return new ConnectionPoint3D(transformed, regionId);
   }

   public ConnectionPoint3D applyInverseTransform(Transform transform)
   {
      Point3D transformed = new Point3D(this);
      transformed.applyInverseTransform(transform);
      return new ConnectionPoint3D(transformed, regionId);
   }
}
