package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class TransferStateEndRecursionMatrix extends DenseMatrix64F
{
   public TransferStateEndRecursionMatrix()
   {
      super(4, 1);
   }

   public void reset()
   {
      zero();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, double omega0, boolean useInitialICP)
   {
      compute(doubleSupportDurations.get(0).getDoubleValue(), omega0, useInitialICP);
   }

   public void compute(double doubleSupportDuration, double omega0, boolean useInitialICP)
   {
      double stateRecursion = Math.exp(-omega0 * doubleSupportDuration);

      zero();
      if (!useInitialICP)
      {
         set(0, 0, stateRecursion);
         set(1, 0, omega0 * stateRecursion);
      }
      set(2, 0, 1.0);
      set(3, 0, omega0);
   }

}

