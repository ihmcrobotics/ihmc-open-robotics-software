package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class TransferPreviousExitCMPProjectionMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable omega;
   private final DoubleYoVariable doubleSupportSplitRatio;

   public TransferPreviousExitCMPProjectionMatrix(DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio)
   {
      super(4, 1);

      this.omega = omega;
      this.doubleSupportSplitRatio = doubleSupportSplitRatio;
   }

   public void reset()
   {
      zero();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations)
   {
      compute(doubleSupportDurations.get(0).getDoubleValue());
   }

   public void compute(double doubleSupportDuration)
   {
      zero();

      double initialDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * doubleSupportDuration;
      double initialDoubleSupportProjection = Math.exp(-omega.getDoubleValue() * initialDoubleSupportDuration);

      set(0, 0, 1.0 - initialDoubleSupportProjection);
      set(1, 0, -omega.getDoubleValue() * initialDoubleSupportProjection);
   }
}
