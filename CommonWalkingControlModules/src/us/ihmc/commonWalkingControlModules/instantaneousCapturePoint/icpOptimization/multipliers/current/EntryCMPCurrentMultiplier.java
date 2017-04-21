package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.SwingEntryCMPMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.TransferEntryCMPMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.List;

public class EntryCMPCurrentMultiplier
{
   private final CubicMatrix cubicMatrix;
   private final CubicDerivativeMatrix cubicDerivativeMatrix;

   private final TransferEntryCMPMatrix transferEntryCMPMatrix;
   private final SwingEntryCMPMatrix swingEntryCMPMatrix;

   private final List<DoubleYoVariable> transferSplitFractions;

   public final DoubleYoVariable startOfSplineTime;
   public final DoubleYoVariable endOfSplineTime;
   public final DoubleYoVariable totalTrajectoryTime;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   private final boolean givenCubicMatrix;
   private final boolean givenCubicDerivativeMatrix;
   private final boolean clipTime;

   public EntryCMPCurrentMultiplier(List<DoubleYoVariable> swingSplitFractions, List<DoubleYoVariable> transferSplitFractions, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime,
         DoubleYoVariable totalTrajectoryTime, String yoNamePrefix, boolean clipTime, YoVariableRegistry registry)
   {
      this(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, totalTrajectoryTime, null, null, yoNamePrefix, clipTime, registry);
   }

   public EntryCMPCurrentMultiplier(List<DoubleYoVariable> swingSplitFractions, List<DoubleYoVariable> transferSplitFractions, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime,
         DoubleYoVariable totalTrajectoryTime, CubicMatrix cubicMatrix, CubicDerivativeMatrix cubicDerivativeMatrix, String yoNamePrefix, boolean clipTime,
         YoVariableRegistry registry)
   {
      positionMultiplier = new DoubleYoVariable(yoNamePrefix + "EntryCMPCurrentMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable(yoNamePrefix + "EntryCMPCurrentVelocityMultiplier", registry);

      this.transferSplitFractions = transferSplitFractions;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;
      this.totalTrajectoryTime = totalTrajectoryTime;

      this.clipTime = clipTime;

      if (cubicMatrix == null)
      {
         this.cubicMatrix = new CubicMatrix();
         givenCubicMatrix = false;
      }
      else
      {
         this.cubicMatrix = cubicMatrix;
         givenCubicMatrix = true;
      }

      if (cubicDerivativeMatrix == null)
      {
         this.cubicDerivativeMatrix = new CubicDerivativeMatrix();
         givenCubicDerivativeMatrix = false;
      }
      else
      {
         this.cubicDerivativeMatrix = cubicDerivativeMatrix;
         givenCubicDerivativeMatrix = true;
      }

      transferEntryCMPMatrix = new TransferEntryCMPMatrix(swingSplitFractions, transferSplitFractions);
      swingEntryCMPMatrix = new SwingEntryCMPMatrix(startOfSplineTime);
   }

   public void reset()
   {
      positionMultiplier.setToNaN();
      velocityMultiplier.setToNaN();
   }

   public double getPositionMultiplier()
   {
      return positionMultiplier.getDoubleValue();
   }

   public double getVelocityMultiplier()
   {
      return velocityMultiplier.getDoubleValue();
   }

   public void compute(int numberOfFootstepsToConsider,
         List<DoubleYoVariable> singleSupportDurations, List<DoubleYoVariable> doubleSupportDurations,
         double timeInState, boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      double positionMultiplier, velocityMultiplier;
      if (isInTransfer)
      {
         positionMultiplier = computeInTransfer(numberOfFootstepsToConsider, singleSupportDurations, doubleSupportDurations, timeInState, useTwoCMPs, omega0);
      }
      else
      {
         if (useTwoCMPs)
            positionMultiplier = computeSegmentedSwing(timeInState, omega0);
         else
            positionMultiplier = computeInSwingOneCMP(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
      }
      this.positionMultiplier.set(positionMultiplier);

      if (isInTransfer)
      {
         velocityMultiplier = computeInTransferVelocity();
      }
      else
      {
         if (useTwoCMPs)
            velocityMultiplier = computeSegmentedSwingVelocity(timeInState, omega0);
         else
            velocityMultiplier = computeInSwingOneCMPVelocity(omega0);
      }

      this.velocityMultiplier.set(velocityMultiplier);
   }




   private double computeInTransfer(int numberOfFootstepsToConsider,
         List<DoubleYoVariable> singleSupportDurations, List<DoubleYoVariable> doubleSupportDurations,
         double timeInState, boolean useTwoCMPs, double omega0)
   {
      transferEntryCMPMatrix.compute(numberOfFootstepsToConsider, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega0);

      double splineDuration = doubleSupportDurations.get(0).getDoubleValue();

      if (!givenCubicDerivativeMatrix)
      {
         cubicDerivativeMatrix.setSegmentDuration(splineDuration);
         cubicDerivativeMatrix.update(timeInState);
      }

      if (!givenCubicMatrix)
      {
         cubicMatrix.setSegmentDuration(splineDuration);
         cubicMatrix.update(timeInState);
      }

      CommonOps.mult(cubicMatrix, transferEntryCMPMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeInTransferVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, transferEntryCMPMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }



   private double computeInSwingOneCMP(List<DoubleYoVariable> singleSupportDurations, List<DoubleYoVariable> doubleSupportDurations,
         double timeInState, double omega0)
   {
      double timeRemaining = singleSupportDurations.get(0).getDoubleValue() - timeInState;
      double nextTransferOnCurrent = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

      if (clipTime)
         timeRemaining = Math.max(timeRemaining, 0.0);

      return 1.0 - Math.exp(-omega0 * (nextTransferOnCurrent + timeRemaining));
   }

   private double computeInSwingOneCMPVelocity(double omega0)
   {
      return omega0 * (positionMultiplier.getDoubleValue() - 1.0);
   }




   private double computeSegmentedSwing(double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeSwingFirstSegment(timeInState, omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeSwingThirdSegment();
      else
         return computeSwingSecondSegment(timeInState, omega0);
   }




   private double computeSwingFirstSegment(double timeInState, double omega0)
   {
      return 1.0 - Math.exp(omega0 * timeInState);
   }

   private double computeSwingSecondSegment(double timeInState, double omega0)
   {
      swingEntryCMPMatrix.compute(omega0);

      double timeInSpline = timeInState - startOfSplineTime.getDoubleValue();
      double splineDuration = endOfSplineTime.getDoubleValue() - startOfSplineTime.getDoubleValue();

      if (!givenCubicMatrix)
      {
         cubicMatrix.setSegmentDuration(splineDuration);
         cubicMatrix.update(timeInSpline);
      }

      if (!givenCubicDerivativeMatrix)
      {
         cubicDerivativeMatrix.setSegmentDuration(splineDuration);
         cubicDerivativeMatrix.update(timeInSpline);
      }

      CommonOps.mult(cubicMatrix, swingEntryCMPMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeSwingThirdSegment()
   {
      return 0.0;
   }




   private double computeSegmentedSwingVelocity(double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeSwingFirstSegmentVelocity(omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeSwingThirdSegmentVelocity();
      else
         return computeSwingSecondSegmentVelocity();
   }




   private double computeSwingFirstSegmentVelocity(double omega0)
   {
      return omega0 * (positionMultiplier.getDoubleValue() - 1.0);
   }

   private double computeSwingSecondSegmentVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, swingEntryCMPMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeSwingThirdSegmentVelocity()
   {
      return 0.0;
   }
}
