package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class SwingInitialICPMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable startOfSplineTime;

   public SwingInitialICPMatrix(DoubleYoVariable startOfSplineTime)
   {
      super(4, 1);

      this.startOfSplineTime = startOfSplineTime;
   }

   public void reset()
   {
      zero();
   }

   public void compute(double omega0)
   {
      double projection = Math.exp(omega0 * startOfSplineTime.getDoubleValue());

      zero();
      set(0, 0, projection);
      set(1, 0, omega0 * projection);
   }
}

