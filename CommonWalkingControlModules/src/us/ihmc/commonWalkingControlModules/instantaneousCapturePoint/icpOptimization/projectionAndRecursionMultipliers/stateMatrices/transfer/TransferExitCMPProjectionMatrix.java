package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class TransferExitCMPProjectionMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable omega;
   private final DoubleYoVariable doubleSupportSplitRatio;

   public TransferExitCMPProjectionMatrix(DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio)
   {
      super(4, 1);

      this.omega = omega;
      this.doubleSupportSplitRatio = doubleSupportSplitRatio;
   }

   public void reset()
   {
      zero();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, boolean useTwoCMPs)
   {
      compute(doubleSupportDurations.get(0).getDoubleValue(), useTwoCMPs);
   }

   public void compute(double doubleSupportDuration, boolean useTwoCMPs)
   {
      double initialDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * doubleSupportDuration;
      double endOfDoubleSupportDuration = (1.0 - doubleSupportSplitRatio.getDoubleValue()) * doubleSupportDuration;

      double initialDoubleSupportProjection = Math.exp(-omega.getDoubleValue() * initialDoubleSupportDuration);
      double endOfDoubleSupportProjection = Math.exp(-omega.getDoubleValue() * endOfDoubleSupportDuration);

      zero();

      if (!useTwoCMPs)
      {
         double totalProjection = initialDoubleSupportProjection * (1.0 - endOfDoubleSupportProjection);

         set(0, 0, totalProjection);
         set(1, 0, omega.getDoubleValue() * totalProjection);

         set(3, 0, -omega.getDoubleValue());
      }
   }
}
