package us.ihmc.robotEnvironmentAwareness.geometry;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

public class VectorMean extends Vector3D
{
   private static final long serialVersionUID = 2936790417418842327L;

   private int sampleSize = 0;

   public VectorMean()
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
      this.addX((x - this.getX()) * nInv);
      this.addY((y - this.getY()) * nInv);
      this.addZ((z - this.getZ()) * nInv);
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
