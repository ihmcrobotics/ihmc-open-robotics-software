package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class SwingInitialICPMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable startOfSplineTime;
   private final boolean blendFromInitial;

   public SwingInitialICPMatrix(DoubleYoVariable startOfSplineTime, boolean blendFromInitial)
   {
      super(4, 1);

      this.startOfSplineTime = startOfSplineTime;
      this.blendFromInitial = blendFromInitial;
   }

   public void reset()
   {
      zero();
   }

   public void compute(double omega0)
   {
      if (blendFromInitial)
      {
         zero();
      }
      else
      {
         // project forward from the initial ICP state
         double projection = Math.exp(omega0 * startOfSplineTime.getDoubleValue());

         zero();
         set(0, 0, projection);
         set(1, 0, omega0 * projection);
      }
   }
}

