package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer.TransferPreviousExitCMPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class PreviousExitCMPProjectionMultiplier
{
   private final CubicMatrix cubicMatrix;
   private final CubicDerivativeMatrix cubicDerivativeMatrix;
   private final TransferPreviousExitCMPProjectionMatrix transferPreviousExitCMPProjectionMatrix;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   public PreviousExitCMPProjectionMultiplier(YoVariableRegistry registry, DoubleYoVariable doubleSupportSplitRatio)
   {
      positionMultiplier = new DoubleYoVariable("PreviousExitCMPProjectionMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable("PreviousExitCMPProjectionVelocityMultiplier", registry);

      cubicMatrix = new CubicMatrix();
      cubicDerivativeMatrix = new CubicDerivativeMatrix();
      transferPreviousExitCMPProjectionMatrix = new TransferPreviousExitCMPProjectionMatrix(doubleSupportSplitRatio);
   }

   public void reset()
   {
      positionMultiplier.set(0.0);
      velocityMultiplier.set(0.0);
   }

   public double getPositionMultiplier()
   {
      return positionMultiplier.getDoubleValue();
   }

   public double getVelocityMultiplier()
   {
      return velocityMultiplier.getDoubleValue();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, double timeRemaining, boolean isInTransfer, double omega0, boolean useInitialICP)
   {
      double positionMultiplier, velocityMultiplier;
      if (isInTransfer)
      {
         positionMultiplier = computeInTransfer(doubleSupportDurations, timeRemaining, omega0, useInitialICP);
         velocityMultiplier = computeInTransferVelocity();
      }
      else
      {
         positionMultiplier = 0.0;
         velocityMultiplier = 0.0;
      }

      this.positionMultiplier.set(positionMultiplier);
      this.velocityMultiplier.set(velocityMultiplier);
   }

   private double computeInTransfer(ArrayList<DoubleYoVariable> doubleSupportDurations, double timeRemaining, double omega0, boolean useInitialICP)
   {
      transferPreviousExitCMPProjectionMatrix.compute(doubleSupportDurations, omega0, useInitialICP);

      double splineDuration = doubleSupportDurations.get(0).getDoubleValue();

      cubicDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicDerivativeMatrix.update(timeRemaining);
      cubicMatrix.setSegmentDuration(splineDuration);
      cubicMatrix.update(timeRemaining);

      CommonOps.mult(cubicMatrix, transferPreviousExitCMPProjectionMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeInTransferVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, transferPreviousExitCMPProjectionMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }
}
