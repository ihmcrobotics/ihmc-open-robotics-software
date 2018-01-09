package us.ihmc.robotEnvironmentAwareness.geometry;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

public class PointMean extends Point3D
{
   private static final long serialVersionUID = -3110940850302600107L;

   private int sampleSize = 0;

   public PointMean()
   {
   }

   public void update(Tuple3DBasics tuple)
   {
      update(tuple.getX(), tuple.getY(), tuple.getZ());
   }

   public void update(Tuple3DBasics tuple, int updateSize)
   {
      update(tuple.getX(), tuple.getY(), tuple.getZ(), updateSize);
   }

   public void update(double x, double y, double z)
   {
      update(x, y, z, 1);
   }

   public void update(double x, double y, double z, int updateSize)
   {
      sampleSize += updateSize;
      double nInv = (double) updateSize / (double) sampleSize;
      this.setX(this.getX() + (x - this.getX()) * nInv);
      this.setY(this.getY() + (y - this.getY()) * nInv);
      this.setZ(this.getZ() + (z - this.getZ()) * nInv);
   }

   public void clear()
   {
      sampleSize = 0;
      set(0.0, 0.0, 0.0);
   }

   public int getNumberOfSamples()
   {
      return sampleSize;
   }
}
