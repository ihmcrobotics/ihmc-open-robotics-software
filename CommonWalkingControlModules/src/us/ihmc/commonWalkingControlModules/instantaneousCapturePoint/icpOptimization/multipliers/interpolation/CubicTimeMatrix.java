package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation;

import org.ejml.data.DenseMatrix64F;

public class CubicTimeMatrix extends DenseMatrix64F
{
   public CubicTimeMatrix()
   {
      super(1, 4);
   }

   public void setCurrentTime(double t)
   {
      set(0, 0, Math.pow(t, 3.0));
      set(0, 1, Math.pow(t, 2.0));
      set(0, 2, t);
      set(0, 3, 1.0);
   }
}
