package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.NewCubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.NewCubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.SwingStateEndMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.TransferStateEndMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.List;

public class StateEndCurrentMultiplier
{
   private final NewCubicMatrix cubicMatrix;
   private final NewCubicDerivativeMatrix cubicDerivativeMatrix;

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

   public StateEndCurrentMultiplier(List<DoubleYoVariable> swingSplitFractions, List<DoubleYoVariable> transferSplitFractions,
         DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, String yoNamePrefix, boolean clipTime, YoVariableRegistry registry)
   {
      this(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, null, null, yoNamePrefix, clipTime, registry);
   }

   public StateEndCurrentMultiplier(List<DoubleYoVariable> swingSplitFractions, List<DoubleYoVariable> transferSplitFractions,
         DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, NewCubicMatrix cubicMatrix, NewCubicDerivativeMatrix cubicDerivativeMatrix,
         String yoNamePrefix, boolean clipTime, YoVariableRegistry registry)
   {
      positionMultiplier = new DoubleYoVariable(yoNamePrefix + "StateEndCurrentMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable(yoNamePrefix + "StateEndCurrentVelocityMultiplier", registry);

      this.swingSplitFractions = swingSplitFractions;
      this.transferSplitFractions = transferSplitFractions;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;

      this.clipTime = clipTime;

      if (cubicMatrix == null)
      {
         this.cubicMatrix = new NewCubicMatrix();
         givenCubicMatrix = false;
      }
      else
      {
         this.cubicMatrix = cubicMatrix;
         givenCubicMatrix = true;
      }

      if (cubicDerivativeMatrix == null)
      {
         this.cubicDerivativeMatrix = new NewCubicDerivativeMatrix();
         givenCubicDerivativeMatrix = false;
      }
      else
      {
         this.cubicDerivativeMatrix = cubicDerivativeMatrix;
         givenCubicDerivativeMatrix = true;
      }

      transferStateEndMatrix = new TransferStateEndMatrix(swingSplitFractions, transferSplitFractions);
      swingStateEndMatrix = new SwingStateEndMatrix(swingSplitFractions, transferSplitFractions, endOfSplineTime);
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

      positionMultiplier.set(Math.exp(-omega0 * (nextTransferOnCurrent + timeRemaining)));
   }


   public void computeInSwingOneCMPVelocity(double omega0)
   {
      velocityMultiplier.set(omega0 * positionMultiplier.getDoubleValue());
   }




   public void computeSwingSegmented(List<DoubleYoVariable> singleSupportDurations, List<DoubleYoVariable> doubleSupportDurations, double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         computeSwingFirstSegment();
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         computeSwingThirdSegment(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
      else
         computeSwingSecondSegment(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
   }

   public void computeSwingFirstSegment()
   {
      positionMultiplier.set(0.0);
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
         computeSwingFirstSegmentVelocity();
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         computeSwingThirdSegmentVelocity(omega0);
      else
         computeSwingSecondSegmentVelocity();
   }

   public void computeSwingFirstSegmentVelocity()
   {
      velocityMultiplier.set(0.0);
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
