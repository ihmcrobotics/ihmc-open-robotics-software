package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class TransferPreviousExitCMPProjectionMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable doubleSupportSplitRatio;

   public TransferPreviousExitCMPProjectionMatrix(DoubleYoVariable doubleSupportSplitRatio)
   {
      super(4, 1);

      this.doubleSupportSplitRatio = doubleSupportSplitRatio;
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
      zero();

      double initialDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * doubleSupportDuration;
      double initialDoubleSupportProjection = Math.exp(-omega0 * initialDoubleSupportDuration);

      if (!useInitialICP)
      {
         set(0, 0, 1.0 - initialDoubleSupportProjection);
         set(1, 0, -omega0 * initialDoubleSupportProjection);
      }
      else
      {
         set(1, 0, -omega0);
      }
   }
}
