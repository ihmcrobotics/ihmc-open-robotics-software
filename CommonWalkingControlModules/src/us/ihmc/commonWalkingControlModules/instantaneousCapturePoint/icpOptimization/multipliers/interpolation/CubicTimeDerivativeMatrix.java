package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation;

import org.ejml.data.DenseMatrix64F;

public class CubicTimeDerivativeMatrix extends DenseMatrix64F
{
   public CubicTimeDerivativeMatrix()
   {
      super(1, 4);
   }

   public void setCurrentTime(double t)
   {
      set(0, 0, 3.0 * Math.pow(t, 2.0));
      set(0, 1, 2.0 * t);
      set(0, 2, 1.0);
      set(0, 3, 0.0);
   }
}
