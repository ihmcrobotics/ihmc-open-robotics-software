package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation;

import org.ejml.data.DenseMatrix64F;

public class CubicSplineCoefficientMatrix extends DenseMatrix64F
{
   public CubicSplineCoefficientMatrix()
   {
      super(4, 4);
   }

   public CubicSplineCoefficientMatrix(double duration)
   {
      this();

      setSegmentDuration(duration);
   }

   public void setSegmentDuration(double duration)
   {
      zero();

      // first row
      set(0, 0, 2.0 / Math.pow(duration, 3.0));
      set(0, 1, 1.0 / Math.pow(duration, 2.0));
      set(0, 2, -2.0 / Math.pow(duration, 3.0));
      set(0, 3, 1.0 / Math.pow(duration, 2.0));

      set(1, 0, -3.0 / Math.pow(duration, 2.0));
      set(1, 1, -2.0 / duration);
      set(1, 2, 3.0 / Math.pow(duration, 2.0));
      set(1, 3, -1 / duration);

      set(2, 1, 1.0);

      set(3, 0, 1.0);
   }
}
