package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.interpolation.EfficientCubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.interpolation.EfficientCubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.stateMatrices.swing.SwingExitCMPMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.stateMatrices.transfer.TransferExitCMPMatrix;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

/**
 * Multiplier of the stance exit CMP. Used to compute the desired current ICP location from the
 * recursively found end-of-state ICP corner point/
 */
public class ExitCMPCurrentMultiplier
{
   /** Cubic spline matrix that multiplies the boundary conditions to compute the current value. */
   private final EfficientCubicMatrix cubicMatrix;
   /** Cubic spline matrix that multiplies the boundary conditions to compute the current derivative value. */
   private final EfficientCubicDerivativeMatrix cubicDerivativeMatrix;

   /** Boundary conditions matrix for the exit CMP when in transfer. */
   private final TransferExitCMPMatrix transferExitCMPMatrix;
   /** Boundary conditions matrix for the exit CMP when in swing. */
   private final SwingExitCMPMatrix swingExitCMPMatrix;

   /** List of transfer split fractions that repartition the transfer phase around the corner point. */
   private final List<YoDouble> transferSplitFractions;
   /** List of swing split fractions that repartition the swing phase around the corner point. */
   private final List<YoDouble> swingSplitFractions;

   /** time in swing state for the start of using the spline */
   public final YoDouble startOfSplineTime;
   /** time in swing state for the end of using the spline */
   public final YoDouble endOfSplineTime;

   /** data holder for multiplied values */
   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   /** multiplier of the exit CMP to compute the current ICP location. */
   private final YoDouble positionMultiplier;
   /** multiplier of the exit CMP to compute the current ICP velocity. */
   private final YoDouble velocityMultiplier;

   /** whether or not the cubic matrix needs to be updated inside this class or is updated outside it. */
   private final boolean givenCubicMatrix;
   /** whether or not the cubic derivative matrix needs to be updated inside this class or is updated outside it. */
   private final boolean givenCubicDerivativeMatrix;
   /** whether or not to clip the time remaining to always be position */
   private final boolean clipTime;
   private final boolean blendFromInitial;

   private final double blendingFraction;
   private final double minimumBlendingTime;

   public ExitCMPCurrentMultiplier( List<YoDouble> swingSplitFractions, List<YoDouble> transferSplitFractions,
         YoDouble startOfSplineTime, YoDouble endOfSplineTime, String yoNamePrefix, boolean clipTime, boolean blendFromInitial,
         double blendingFraction, double minimumBlendingTime, YoVariableRegistry registry)
   {
      this(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, null, null, yoNamePrefix, clipTime, blendFromInitial,
            blendingFraction, minimumBlendingTime, registry);
   }

   public ExitCMPCurrentMultiplier(List<YoDouble> swingSplitFractions, List<YoDouble> transferSplitFractions,
         YoDouble startOfSplineTime, YoDouble endOfSplineTime, EfficientCubicMatrix cubicMatrix,
         EfficientCubicDerivativeMatrix cubicDerivativeMatrix, String yoNamePrefix, boolean clipTime, boolean blendFromInitial, double blendingFraction,
         double minimumBlendingTime, YoVariableRegistry registry)
   {
      positionMultiplier = new YoDouble(yoNamePrefix + "ExitCMPCurrentMultiplier", registry);
      velocityMultiplier = new YoDouble(yoNamePrefix + "ExitCMPCurrentVelocityMultiplier", registry);

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

      transferExitCMPMatrix = new TransferExitCMPMatrix(swingSplitFractions, transferSplitFractions);
      swingExitCMPMatrix = new SwingExitCMPMatrix(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, blendFromInitial,
            minimumBlendingTime);
   }

   /**
    * Resets the multiplier values to NaN. Done at every tick.
    */
   public void reset()
   {
      positionMultiplier.setToNaN();
      velocityMultiplier.setToNaN();
   }

   /**
    * Gets the value to multiply the exit CMP location by to compute the current ICP location.
    *
    * @return position multiplier.
    */
   public double getPositionMultiplier()
   {
      return positionMultiplier.getDoubleValue();
   }

   /**
    * Gets the value to multiply the exit CMP location by to compute the current ICP velocity.
    *
    * @return velocity multiplier.
    */
   public double getVelocityMultiplier()
   {
      return velocityMultiplier.getDoubleValue();
   }

   /**
    * Computes the exit CMP multiplier. Must be called every control tick.
    *
    * @param numberOfFootstepsToConsider total number of footsteps the optimization is considering.
    * @param singleSupportDurations vector of single support durations.
    * @param doubleSupportDurations vector of double support durations.
    * @param timeInState time in the current state.
    * @param useTwoCMPs whether or not to use two CMPs in the ICP plan.
    * @param isInTransfer whether or not the robot is currently in the transfer phase.
    * @param omega0 natural frequency of the inverted pendulum.
    */
   public void compute(int numberOfFootstepsToConsider,
         List<YoDouble> singleSupportDurations, List<YoDouble> doubleSupportDurations,
         double timeInState, boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      if (isInTransfer)
      {
         computeInTransfer(numberOfFootstepsToConsider, singleSupportDurations, doubleSupportDurations, timeInState, useTwoCMPs, omega0);
      }
      else
      {
         if (useTwoCMPs)
            computeSegmentedSwing(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
         else
            computeInSwingOneCMP();
      }

      if (isInTransfer)
      {
         computeInTransferVelocity();
      }
      else
      {
         if (useTwoCMPs)
            computeSegmentedSwingVelocity(timeInState, omega0);
         else
            computeInSwingOneCMPVelocity();
      }
   }




   /**
    * Computes the position multiplier when in the transfer phase. During this phase, the trajectory is a cubic spline,
    * so this is used to calculate the position multiplier.
    *
    * @param numberOfFootstepsToConsider number of footsteps to consider by the ICP recursive dynamics.
    * @param singleSupportDurations vector of single support durations
    * @param doubleSupportDurations vector of double support durations
    * @param timeInState time in the transfer state
    * @param useTwoCMPs whether or not to use two CMPs in each foot to calculate the ICP plan.
    * @param omega0 natural frequency of the inverted pendulum.
    */
   public void computeInTransfer(int numberOfFootstepsToConsider,
         List<YoDouble> singleSupportDurations, List<YoDouble> doubleSupportDurations,
         double timeInState, boolean useTwoCMPs, double omega0)
   {
      transferExitCMPMatrix.compute(numberOfFootstepsToConsider, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega0);

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

      CommonOps.mult(cubicMatrix, transferExitCMPMatrix, matrixOut);

      positionMultiplier.set(matrixOut.get(0, 0));
   }

   /**
    * Computes the position multiplier when in the transfer phase. During this phase, the trajectory is a
    * cubic spline, so this is used to calculate the position multiplier.
    */
   public void computeInTransferVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, transferExitCMPMatrix, matrixOut);

      velocityMultiplier.set(matrixOut.get(0, 0));
   }




   /**
    * Computes the position multiplier in the swing phase when using one CMP in each foot. As there is only one
    * CMP, the exit CMP is not considered, requiring this multiplier to be zero.
    */
   public void computeInSwingOneCMP()
   {
      positionMultiplier.set(0.0);
   }

   /**
    * Computes the velocity multiplier in the swing phase when using one CMP in each foot. As there is only one
    * CMP, the exit CMP is not considered, requiring this multiplier to be zero.
    */
   public void computeInSwingOneCMPVelocity()
   {
      velocityMultiplier.set(0.0);
   }





   /**
    * Computes the position multiplier in the swing phase when using two CMPs in each foot.
    *
    * @param singleSupportDurations vector of single support durations
    * @param doubleSupportDurations vector of double support durations
    * @param timeInState time in the swing state.
    * @param omega0 natural frequency of the inverted pendulum
    */
   public void computeSegmentedSwing(List<YoDouble> singleSupportDurations, List<YoDouble> doubleSupportDurations,
         double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         computeSwingFirstSegment(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         computeSwingThirdSegment(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
      else
         computeSwingSecondSegment(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
   }

   /**
    * Computes the position multiplier when in the third segment in the swing phase. During this
    * segment, the ICP plan is on the entry CMP, so this value is 0.0.
    */
   public void computeSwingFirstSegment(List<YoDouble> singleSupportDurations, List<YoDouble> doubleSupportDurations, double timeInState, double omega0)
   {
      if (blendFromInitial)
      {
         double projectionMultiplier = 0.0;

         double nextTransferOnExitCMP = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
         double swingTimeOnExitCMP = (1.0 - swingSplitFractions.get(0).getDoubleValue()) * singleSupportDurations.get(0).getDoubleValue();
         double swingTimeOnEntryCMP = swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();

         double timeOnExitCMP = nextTransferOnExitCMP + swingTimeOnExitCMP;

         double recursionMultiplier = Math.exp(omega0 * (timeInState - swingTimeOnEntryCMP)) * (1.0 - Math.exp(-omega0 * timeOnExitCMP));

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

   /**
    * Computes the position multiplier when in the second segment in the swing phase. During this
    * segment, the trajectory is a cubic spline, so this is used to calculate the position
    * multiplier.
    *
    * @param singleSupportDurations vector of single support durations
    * @param doubleSupportDurations vector of double support durations
    * @param timeInState time in the swing state.
    * @param omega0 natural frequency of the inverted pendulum.
    */
   public void computeSwingSecondSegment(List<YoDouble> singleSupportDurations, List<YoDouble> doubleSupportDurations,
         double timeInState, double omega0)
   {
      swingExitCMPMatrix.compute(singleSupportDurations, doubleSupportDurations, omega0);

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

      CommonOps.mult(cubicMatrix, swingExitCMPMatrix, matrixOut);

      positionMultiplier.set(matrixOut.get(0, 0));
   }

   /**
    * Computes the position multiplier in the third segment of the swing phase when using two CMPs in each foot. The desired
    * ICP position is calculated as<br>
    *    &xi; = e<sup>-&omega; t<sub>r</sub></sup> &xi;<sub>end</sub> + (1.0 - e<sup>-&omega; t<sub>r</sub></sup>) r<sub>cmp,T</sub><br>
    * where &xi;<sub>end</sub> is the ICP location at the end of the swing state and t<sub>r</sub> is the time remaining in the state.
    * &xi;<sub>end</sub> is found from the active corner point by<br>
    *    &xi;<sub>end</sub> = e<sup>-&omega; t<sub>iniDS,1</sub></sup> &xi;<sub>HT</sub> + (1.0 - e<sup>-&omega; t<sub>iniDS,1</sub></sup>) r<sub>cmp,T</sub>,<br>
    * where t<sub>iniDS,1</sub> is the next transfer duration spent on this CMP and &xi;<sub>HT</sub> is the next entry CMP. From this,
    * &gamma;<sub>T</sub> can be found by<br>
    *   &gamma;<sub>T</sub> = 1.0 - e<sup>-&omega; (t<sub>r</sub> + t<sub>iniDS,1</sub>)</sup>
    *
    * @param singleSupportDurations vector of single support durations.
    * @param doubleSupportDurations vector of double support durations.
    * @param timeInState time in the swing state.
    * @param omega0 natural frequency of the inverted pendulum.
    */
   public void computeSwingThirdSegment(List<YoDouble> singleSupportDurations, List<YoDouble> doubleSupportDurations,
         double timeInState, double omega0)
   {
      double timeRemaining = singleSupportDurations.get(0).getDoubleValue() - timeInState;
      double nextTransferOnExit = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

      if (clipTime)
         timeRemaining = Math.max(0.0, timeRemaining);

      positionMultiplier.set(1.0 - Math.exp(-omega0 * (nextTransferOnExit + timeRemaining)));
   }





   /**
    * Computes the velocity multiplier in the swing phase when using two CMPs in each foot.
    *
    * @param timeInState time in the swing state.
    * @param omega0 natural frequency of the inverted pendulum
    */
   public void computeSegmentedSwingVelocity(double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         computeSwingFirstSegmentVelocity(omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         computeSwingThirdSegmentVelocity(omega0);
      else
         computeSwingSecondSegmentVelocity();
   }

   /**
    * Computes the velocity multiplier when in the third segment in the swing phase. During this
    * segment, the ICP plan is on the entry CMP, so this value is 0.0.
    */
   public void computeSwingFirstSegmentVelocity(double omega0)
   {
      velocityMultiplier.set(omega0 * positionMultiplier.getDoubleValue());
   }

   /**
    * Computes the velocity multiplier when in the second segment in the swing phase. During this
    * segment, the trajectory is a cubic spline, so this is used to calculate the velocity
    * multiplier.
    */
   public void computeSwingSecondSegmentVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, swingExitCMPMatrix, matrixOut);

      velocityMultiplier.set(matrixOut.get(0, 0));
   }

   /**
    * Computes the velocity multiplier in the third segment of the swing phase.
    *
    * @param omega0 natural frequency of the inverted pendulum.
    */
   public void computeSwingThirdSegmentVelocity(double omega0)
   {
      velocityMultiplier.set(omega0 * (positionMultiplier.getDoubleValue() - 1.0));
   }
}
