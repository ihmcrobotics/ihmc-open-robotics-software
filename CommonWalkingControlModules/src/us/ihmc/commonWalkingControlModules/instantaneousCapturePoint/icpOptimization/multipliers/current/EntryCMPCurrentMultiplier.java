package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.EfficientCubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.EfficientCubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.SwingEntryCMPMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.TransferEntryCMPMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.List;

/**
 * Multiplier of the stance entry CMP. Used to compute the desired current ICP location from the
 * recursively found end-of-state ICP corner point.
 */
public class EntryCMPCurrentMultiplier
{
   /** Cubic spline matrix that multiplies the boundary conditions to compute the current value. */
   private final EfficientCubicMatrix cubicMatrix;
   /** Cubic spline matrix that multiplies the boundary conditions to compute the current derivative value. */
   private final EfficientCubicDerivativeMatrix cubicDerivativeMatrix;

   /** Boundary conditions matrix for the entry CMP when in transfer. */
   private final TransferEntryCMPMatrix transferEntryCMPMatrix;
   /** Boundary conditions matrix for the entry CMP when in swing. */
   private final SwingEntryCMPMatrix swingEntryCMPMatrix;

   /** List of transfer split fractions that repartition the transfer phase around the corner point. */
   private final List<DoubleYoVariable> transferSplitFractions;

   /** time in swing state for the start of using the spline */
   public final DoubleYoVariable startOfSplineTime;
   /** time in swing state for the end of using the spline */
   public final DoubleYoVariable endOfSplineTime;
   /** total duration of the spline in swing */
   public final DoubleYoVariable totalTrajectoryTime;

   /** data holder for multiplied values */
   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   /** multiplier of the entry CMP to compute the current ICP location. */
   private final DoubleYoVariable positionMultiplier;
   /** multiplier of the entry CMP to compute the current ICP velocity. */
   private final DoubleYoVariable velocityMultiplier;

   /** whether or not the cubic matrix needs to be updated inside this class or is updated outside it. */
   private final boolean givenCubicMatrix;
   /** whether or not the cubic derivative matrix needs to be updated inside this class or is updated outside it. */
   private final boolean givenCubicDerivativeMatrix;
   /** whether or not to clip the time remaining to always be position */
   private final boolean clipTime;

   public EntryCMPCurrentMultiplier(List<DoubleYoVariable> swingSplitFractions, List<DoubleYoVariable> transferSplitFractions,
         DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, DoubleYoVariable totalTrajectoryTime, String yoNamePrefix, boolean clipTime,
         YoVariableRegistry registry)
   {
      this(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, totalTrajectoryTime, null, null, yoNamePrefix, clipTime, registry);
   }

   public EntryCMPCurrentMultiplier(List<DoubleYoVariable> swingSplitFractions, List<DoubleYoVariable> transferSplitFractions,
         DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, DoubleYoVariable totalTrajectoryTime, EfficientCubicMatrix cubicMatrix,
         EfficientCubicDerivativeMatrix cubicDerivativeMatrix, String yoNamePrefix, boolean clipTime,
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

      transferEntryCMPMatrix = new TransferEntryCMPMatrix(swingSplitFractions, transferSplitFractions);
      swingEntryCMPMatrix = new SwingEntryCMPMatrix(startOfSplineTime);
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
    * Gets the value to multiply the entry CMP location by to compute the current ICP location.
    *
    * @return position multiplier.
    */
   public double getPositionMultiplier()
   {
      return positionMultiplier.getDoubleValue();
   }

   /**
    * Gets the value to multiply the entry CMP location by to compute the current ICP velocity.
    *
    * @return velocity multiplier.
    */
   public double getVelocityMultiplier()
   {
      return velocityMultiplier.getDoubleValue();
   }

   /**
    * Computes the entry CMP multiplier. Must be called every control tick.
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
            computeSegmentedSwing(timeInState, omega0);
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
            computeSegmentedSwingVelocity(timeInState, omega0);
         else
            computeInSwingOneCMPVelocity(omega0);
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

      positionMultiplier.set(matrixOut.get(0, 0));
   }

   /**
    * Computes the position multiplier when in the transfer phase. During this phase, the trajectory is a
    * cubic spline, so this is used to calculate the position multiplier.
    */
   public void computeInTransferVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, transferEntryCMPMatrix, matrixOut);

      velocityMultiplier.set(matrixOut.get(0, 0));
   }



   /**
    * Computes the position multiplier in the swing phase when using one CMP in each foot. The desired
    * ICP position is calculated as<br>
    *    &xi; = e<sup>-&omega; t<sub>r</sub></sup> &xi;<sub>end</sub> + (1.0 - e<sup>-&omega; t<sub>r</sub></sup>) r<sub>cmp,H</sub><br>
    * where &xi;<sub>end</sub> is the ICP location at the end of the swing state and t<sub>r</sub> is the time remaining in the state.
    * &xi;<sub>end</sub> is found from the active corner point by<br>
    *    &xi;<sub>end</sub> = e<sup>-&omega; t<sub>iniDS,1</sub></sup> &xi;<sub>HT</sub> + (1.0 - e<sup>-&omega; t<sub>iniDS,1</sub></sup>) r<sub>cmp,H</sub>,<br>
    * where t<sub>iniDS,1</sub> is the next transfer duration spent on this CMP and &xi;<sub>HT</sub> is the next entry CMP. From this,
    * &gamma;<sub>H</sub> can be found by<br>
    *   &gamma;<sub>H</sub> = 1.0 - e<sup>-&omega; (t<sub>r</sub> + t<sub>iniDS,1</sub>)</sup>
    *
    * @param singleSupportDurations vector of single support durations.
    * @param doubleSupportDurations vector of double support durations.
    * @param timeInState time in the swing state.
    * @param omega0 natural frequency of the inverted pendulum.
    */
   public void computeInSwingOneCMP(List<DoubleYoVariable> singleSupportDurations, List<DoubleYoVariable> doubleSupportDurations,
         double timeInState, double omega0)
   {
      double timeRemaining = singleSupportDurations.get(0).getDoubleValue() - timeInState;
      double nextTransferOnCurrent = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

      if (clipTime)
         timeRemaining = Math.max(timeRemaining, 0.0);

      positionMultiplier.set(1.0 - Math.exp(-omega0 * (nextTransferOnCurrent + timeRemaining)));
   }

   /**
    * Computes the velocity multiplier in the swing phase when using one CMP in each foot.
    *
    * @param omega0 natural frequency of the inverted pendulum.
    */
   public void computeInSwingOneCMPVelocity(double omega0)
   {
      velocityMultiplier.set(omega0 * (positionMultiplier.getDoubleValue() - 1.0));
   }




   /**
    * Computes the position multiplier in the swing phase when using two CMPs in each foot.
    *
    * @param timeInState time in the swing state.
    * @param omega0 natural frequency of the inverted pendulum
    */
   public void computeSegmentedSwing(double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         computeSwingFirstSegment(timeInState, omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         computeSwingThirdSegment();
      else
         computeSwingSecondSegment(timeInState, omega0);
   }


   /**
    * Computes the position multiplier when in the first segment in the swing phase. During this
    * segment, the ICP plan is on the entry CMP. In this phase, the position is given by <br>
    *    &xi; = e<sup>&omega; t</sup> &xi;<sub>0</sub> + (1.0 - e<sup>&omega; t</sup>) r<sub>cmp,H</sub><br>
    * So the multiplier is simply &gamma;<sub>H</sub> = (1.0 - e<sup>&omega; t</sup>)
    *
    * @param timeInState time in the swing state
    * @param omega0 natural frequency of the inverted pendulum.
    */
   public void computeSwingFirstSegment(double timeInState, double omega0)
   {
      positionMultiplier.set(1.0 - Math.exp(omega0 * timeInState));
   }

   /**
    * Computes the position multiplier when in the second segment in the swing phase. During this
    * segment, the trajectory is a cubic spline, so this is used to calculate the position
    * multiplier.
    *
    * @param timeInState time in the swing state.
    * @param omega0 natural frequency of the inverted pendulum.
    */
   public void computeSwingSecondSegment(double timeInState, double omega0)
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

      positionMultiplier.set(matrixOut.get(0, 0));
   }

   /**
    * Computes the position multiplier when in the third segment in the swing phase. During this
    * segment, the ICP plan is on the exit CMP, so this value is 0.0.
    */
   public void computeSwingThirdSegment()
   {
      positionMultiplier.set(0.0);
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
         computeSwingThirdSegmentVelocity();
      else
         computeSwingSecondSegmentVelocity();
   }

   /**
    * Computes the velocity multiplier when in the first segment in the swing phase. During this
    * segment, the ICP plan is on the entry CMP. In this phase, the position and velocity are given by <br>
    *    &xi; = e<sup>&omega; t</sup> &xi;<sub>0</sub> + (1.0 - e<sup>&omega; t</sup>) r<sub>cmp,H</sub><br>
    *    \dot&xi; = &omega;e<sup>&omega; t</sup> &xi;<sub>0</sub> + -&omega;e<sup>&omega; t</sup> r<sub>cmp,H</sub><br>
    * So the multiplier is simply &omega; * (&gamma;<sub>H</sub> - 1.0), <br>
    * where <br>
    *    &gamma;<sub>H</sub> = (1.0 - e<sup>&omega; t</sup>)
    *
    * @param omega0 natural frequency of the inverted pendulum.
    */
   public void computeSwingFirstSegmentVelocity(double omega0)
   {
      velocityMultiplier.set(omega0 * (positionMultiplier.getDoubleValue() - 1.0));
   }

   /**
    * Computes the velocity multiplier when in the second segment in the swing phase. During this
    * segment, the trajectory is a cubic spline, so this is used to calculate the velocity
    * multiplier.
    */
   public void computeSwingSecondSegmentVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, swingEntryCMPMatrix, matrixOut);

      velocityMultiplier.set(matrixOut.get(0, 0));
   }

   /**
    * Computes the velocity multiplier when in the third segment in the swing phase. During this
    * segment, the ICP plan is on the exit CMP, so this value is 0.0.
    */
   public void computeSwingThirdSegmentVelocity()
   {
      velocityMultiplier.set(0.0);
   }
}
