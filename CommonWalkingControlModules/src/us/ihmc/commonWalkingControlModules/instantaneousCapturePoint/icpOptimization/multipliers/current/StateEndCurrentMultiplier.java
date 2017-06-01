package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.EfficientCubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.EfficientCubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.SwingStateEndMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.TransferStateEndMatrix;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.List;

public class StateEndCurrentMultiplier
{
   private final EfficientCubicMatrix cubicMatrix;
   private final EfficientCubicDerivativeMatrix cubicDerivativeMatrix;

   private final boolean givenCubicMatrix;
   private final boolean givenCubicDerivativeMatrix;

   private final TransferStateEndMatrix transferStateEndMatrix;
   private final SwingStateEndMatrix swingStateEndMatrix;

   private final List<DoubleYoVariable> swingSplitFractions;
   private final List<DoubleYoVariable> transferSplitFractions;

   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   private final boolean clipTime;
   private final boolean blendFromInitial;
   private final double blendingFraction;
   private final double minimumBlendingTime;

   public StateEndCurrentMultiplier(List<DoubleYoVariable> swingSplitFractions, List<DoubleYoVariable> transferSplitFractions,
         DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, String yoNamePrefix, boolean clipTime, boolean blendFromInitial,
         double blendingFraction, double minimumBlendingTime, YoVariableRegistry registry)
   {
      this(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, null, null, yoNamePrefix, clipTime, blendFromInitial,
            blendingFraction, minimumBlendingTime, registry);
   }

   public StateEndCurrentMultiplier(List<DoubleYoVariable> swingSplitFractions, List<DoubleYoVariable> transferSplitFractions,
         DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, EfficientCubicMatrix cubicMatrix, EfficientCubicDerivativeMatrix cubicDerivativeMatrix,
         String yoNamePrefix, boolean clipTime, boolean blendFromInitial, double blendingFraction, double minimumBlendingTime, YoVariableRegistry registry)
   {
      positionMultiplier = new DoubleYoVariable(yoNamePrefix + "StateEndCurrentMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable(yoNamePrefix + "StateEndCurrentVelocityMultiplier", registry);

      this.swingSplitFractions = swingSplitFractions;
      this.transferSplitFractions = transferSplitFractions;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;

      this.clipTime = clipTime;
      this.blendFromInitial = blendFromInitial;
      this.blendingFraction = blendingFraction;
      this.minimumBlendingTime = minimumBlendingTime;

      if (cubicMatrix == null)
      {
         this.cubicMatrix = new EfficientCubicMatrix();
         givenCubicMatrix = false;
      }
      else
      {
         this.cubicMatrix = cubicMatrix;
         givenCubicMatrix = true;
      }

      if (cubicDerivativeMatrix == null)
      {
         this.cubicDerivativeMatrix = new EfficientCubicDerivativeMatrix();
         givenCubicDerivativeMatrix = false;
      }
      else
      {
         this.cubicDerivativeMatrix = cubicDerivativeMatrix;
         givenCubicDerivativeMatrix = true;
      }

      transferStateEndMatrix = new TransferStateEndMatrix(swingSplitFractions, transferSplitFractions);
      swingStateEndMatrix = new SwingStateEndMatrix(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, blendFromInitial,
            minimumBlendingTime);
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
      if (isInTransfer)
      {
         computeInTransfer(numberOfFootstepsToConsider, singleSupportDurations, doubleSupportDurations, timeInState, useTwoCMPs, omega0);
      }
      else
      {
         if (useTwoCMPs)
            computeSwingSegmented(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
         else
            computeInSwingOneCMP(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
      }

      if (isInTransfer)
      {
         computeInTransferVelocity();
      }
      else
      {
         if (useTwoCMPs)
            computeSwingSegmentedVelocity(timeInState, omega0);
         else
            computeInSwingOneCMPVelocity(omega0);
      }
   }

   public void computeInTransfer(int numberOfFootstepsToConsider,
         List<DoubleYoVariable> singleSupportDurations, List<DoubleYoVariable> doubleSupportDurations,
         double timeInState, boolean useTwoCMPs, double omega0)
   {
      transferStateEndMatrix.compute(numberOfFootstepsToConsider, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega0);

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

      CommonOps.mult(cubicMatrix, transferStateEndMatrix, matrixOut);

      positionMultiplier.set(matrixOut.get(0, 0));
   }

   public void computeInTransferVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, transferStateEndMatrix, matrixOut);

      velocityMultiplier.set(matrixOut.get(0, 0));
   }





   public void computeInSwingOneCMP(List<DoubleYoVariable> singleSupportDurations, List<DoubleYoVariable> doubleSupportDurations,
         double timeInState, double omega0)
   {
      double timeRemaining = singleSupportDurations.get(0).getDoubleValue() - timeInState;
      double nextTransferOnCurrent = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

      if (clipTime)
         timeRemaining = Math.max(timeRemaining, 0.0);

      if (blendFromInitial)
      {
         double recursionMultiplier = Math.exp(-omega0 * (nextTransferOnCurrent + timeRemaining));
         double projectionMultiplier = 0.0;

         double blendingTime = blendingFraction * singleSupportDurations.get(0).getDoubleValue();
         blendingTime = Math.max(blendingTime, minimumBlendingTime);
         double phaseInState = MathTools.clamp(timeInState / blendingTime, 0.0, 1.0);

         double multiplier = InterpolationTools.linearInterpolate(projectionMultiplier, recursionMultiplier, phaseInState);
         positionMultiplier.set(multiplier);
      }
      else
      {
         positionMultiplier.set(Math.exp(-omega0 * (nextTransferOnCurrent + timeRemaining)));
      }
   }


   public void computeInSwingOneCMPVelocity(double omega0)
   {
      velocityMultiplier.set(omega0 * positionMultiplier.getDoubleValue());
   }




   public void computeSwingSegmented(List<DoubleYoVariable> singleSupportDurations, List<DoubleYoVariable> doubleSupportDurations, double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         computeSwingFirstSegment(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         computeSwingThirdSegment(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
      else
         computeSwingSecondSegment(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
   }

   public void computeSwingFirstSegment(List<DoubleYoVariable> singleSupportDurations, List<DoubleYoVariable> doubleSupportDurations,
         double timeInState, double omega0)
   {
      if (blendFromInitial)
      {
         double nextTransferOnExitCMP = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
         double currentSwingOnExitCMP = (1.0 - swingSplitFractions.get(0).getDoubleValue()) * singleSupportDurations.get(0).getDoubleValue();
         double timeOnExitCMP = currentSwingOnExitCMP + nextTransferOnExitCMP;

         double currentSwingOnEntryCMP = swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();

         double recursionMultiplier = Math.exp(omega0 * (timeInState - timeOnExitCMP - currentSwingOnEntryCMP));
         double projectionMultiplier = 0.0;

         double blendingTime = blendingFraction * startOfSplineTime.getDoubleValue();
         blendingTime = Math.max(blendingTime, minimumBlendingTime);
         double phaseInState = MathTools.clamp(timeInState / blendingTime, 0.0, 1.0);

         double multiplier = InterpolationTools.linearInterpolate(projectionMultiplier, recursionMultiplier, phaseInState);
         positionMultiplier.set(multiplier);
      }
      else
      {
         positionMultiplier.set(0.0);
      }
   }

   public void computeSwingSecondSegment(List<DoubleYoVariable> singleSupportDurations, List<DoubleYoVariable> doubleSupportDurations,
         double timeInState, double omega0)
   {
      swingStateEndMatrix.compute(singleSupportDurations, doubleSupportDurations, omega0);

      double timeInSpline = timeInState - startOfSplineTime.getDoubleValue();
      double splineDuration = endOfSplineTime.getDoubleValue() - startOfSplineTime.getDoubleValue();

      if (!givenCubicDerivativeMatrix)
      {
         cubicDerivativeMatrix.setSegmentDuration(splineDuration);
         cubicDerivativeMatrix.update(timeInSpline);
      }
      if (!givenCubicMatrix)
      {
         cubicMatrix.setSegmentDuration(splineDuration);
         cubicMatrix.update(timeInSpline);
      }

      CommonOps.mult(cubicMatrix, swingStateEndMatrix, matrixOut);

      positionMultiplier.set(matrixOut.get(0, 0));
   }

   public void computeSwingThirdSegment(List<DoubleYoVariable> singleSupportDurations, List<DoubleYoVariable> doubleSupportDurations, double timeInState, double omega0)
   {
      double timeRemaining = singleSupportDurations.get(0).getDoubleValue() - timeInState;
      double nextTransferOnExit = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

      if (clipTime)
         timeRemaining = Math.max(timeRemaining, 0.0);

      positionMultiplier.set(Math.exp(-omega0 * (nextTransferOnExit + timeRemaining)));
   }




   public void computeSwingSegmentedVelocity(double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         computeSwingFirstSegmentVelocity(omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         computeSwingThirdSegmentVelocity(omega0);
      else
         computeSwingSecondSegmentVelocity();
   }

   public void computeSwingFirstSegmentVelocity(double omega0)
   {
      velocityMultiplier.set(omega0 * positionMultiplier.getDoubleValue());
   }

   public void computeSwingSecondSegmentVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, swingStateEndMatrix, matrixOut);

      velocityMultiplier.set(matrixOut.get(0, 0));
   }

   public void computeSwingThirdSegmentVelocity(double omega0)
   {
      velocityMultiplier.set(omega0 * positionMultiplier.getDoubleValue());
   }
}
