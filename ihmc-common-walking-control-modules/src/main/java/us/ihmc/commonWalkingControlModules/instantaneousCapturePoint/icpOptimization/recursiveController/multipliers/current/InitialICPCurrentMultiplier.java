package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.interpolation.EfficientCubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.interpolation.EfficientCubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.stateMatrices.swing.SwingInitialICPMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.stateMatrices.transfer.TransferInitialICPMatrix;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

/**
 * Multiplier of the initial ICP value at the beginning of the current state. Used to compute the desired current
 * ICP location from the recursively found end-of-state ICP corner point
 */
public class InitialICPCurrentMultiplier
{
   /** Cubic spline matrix that multiplies the boundary conditions to compute the current value. */
   private final EfficientCubicMatrix cubicMatrix;
   /** Cubic spline matrix that multiplies the boundary conditions to compute the current derivative value. */
   private final EfficientCubicDerivativeMatrix cubicDerivativeMatrix;

   /** whether or not the cubic matrix needs to be updated inside this class or is updated outside it. */
   private final boolean givenCubicMatrix;
   /** whether or not the cubic derivative matrix needs to be updated inside this class or is updated outside it. */
   private final boolean givenCubicDerivativeMatrix;

   /** Boundary conditions matrix for the initial ICP when in transfer. */
   private final TransferInitialICPMatrix transferInitialICPMatrix;
   /** Boundary conditions matrix for the initial ICP when in swing. */
   private final SwingInitialICPMatrix swingInitialICPMatrix;

   /** time in swing state for the start of using the spline */
   public final YoDouble startOfSplineTime;
   /** time in swing state for the end of using the spline */
   public final YoDouble endOfSplineTime;

   /** data holder for multiplied values */
   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   /** multiplier of the initial ICP to compute the current ICP location. */
   private final YoDouble positionMultiplier;
   /** multiplier of the initial ICP to compute the current ICP velocity. */
   private final YoDouble velocityMultiplier;

   private final boolean blendFromInitial;
   private final double blendingFraction;
   private final double minimumBlendingTime;

   public InitialICPCurrentMultiplier(YoDouble startOfSplineTime, YoDouble endOfSplineTime, boolean blendFromInitial, double blendingFraction,
         double minimumBlendingTime, String yoNamePrefix, YoVariableRegistry registry)
   {
      this(startOfSplineTime, endOfSplineTime, null, null, blendFromInitial, blendingFraction, minimumBlendingTime, yoNamePrefix, registry);
   }

   public InitialICPCurrentMultiplier(YoDouble startOfSplineTime, YoDouble endOfSplineTime, EfficientCubicMatrix cubicMatrix,
         EfficientCubicDerivativeMatrix cubicDerivativeMatrix, boolean blendFromInitial, double blendingFraction, double minimumBlendingTime, String yoNamePrefix, YoVariableRegistry registry)
   {
      positionMultiplier = new YoDouble(yoNamePrefix + "InitialICPCurrentMultiplier", registry);
      velocityMultiplier = new YoDouble(yoNamePrefix + "InitialICPCurrentVelocityMultiplier", registry);

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;

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

      transferInitialICPMatrix = new TransferInitialICPMatrix();
      swingInitialICPMatrix = new SwingInitialICPMatrix(startOfSplineTime, blendFromInitial, minimumBlendingTime);
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
    * Gets the value to multiply the initial ICP location by to compute the current ICP location.
    *
    * @return position multiplier.
    */
   public double getPositionMultiplier()
   {
      return positionMultiplier.getDoubleValue();
   }

   /**
    * Gets the value to multiply the initial ICP location by to compute the current ICP velocity.
    *
    * @return velocity multiplier.
    */
   public double getVelocityMultiplier()
   {
      return velocityMultiplier.getDoubleValue();
   }

   /**
    * Computes the initial ICP multiplier. Must be called every control tick.
    *
    * @param singleSupportDurations vector of single support durations.
    * @param doubleSupportDurations vector of double support durations.
    * @param timeInState time in the current state.
    * @param useTwoCMPs whether or not to use two CMPs in the ICP plan.
    * @param isInTransfer whether or not the robot is currently in the transfer phase.
    * @param omega0 natural frequency of the inverted pendulum.
    */
   public void compute(List<YoDouble> singleSupportDurations, List<YoDouble> doubleSupportDurations,
         double timeInState, boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      if (isInTransfer)
      {
         computeInTransfer(doubleSupportDurations, timeInState);
      }
      else
      {
         if (useTwoCMPs)
            computeSwingSegmented(singleSupportDurations, timeInState, omega0);
         else
            computeInSwingOneCMP(singleSupportDurations, timeInState, omega0);
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



   /**
    * Computes the position multiplier when in the transfer phase. During this phase, the trajectory is a cubic spline,
    * so this is used to calculate the position multiplier.
    *
    * @param doubleSupportDurations vector of double support durations
    * @param timeInState time in the transfer state
    */
   public void computeInTransfer(List<YoDouble> doubleSupportDurations, double timeInState)
   {
      transferInitialICPMatrix.compute();

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

      CommonOps.mult(cubicMatrix, transferInitialICPMatrix, matrixOut);

      positionMultiplier.set(matrixOut.get(0, 0));
   }

   /**
    * Computes the position multiplier when in the transfer phase. During this phase, the trajectory is a
    * cubic spline, so this is used to calculate the position multiplier.
    */
   public void computeInTransferVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, transferInitialICPMatrix, matrixOut);

      velocityMultiplier.set(matrixOut.get(0, 0));
   }





   /**
    * Computes the position multiplier in the swing phase when using one CMP in each foot. The desired
    * ICP position computed from the stance entry CMP and the next corner point, not the initial ICP location.
    */
   public void computeInSwingOneCMP(List<YoDouble> singleSupportDurations, double timeInState, double omega0)
   {
      if (blendFromInitial)
      {
         double projectionMultiplier = Math.exp(omega0 * timeInState);
         double recursionMultiplier = 0.0;

         double blendingTime = blendingFraction * singleSupportDurations.get(0).getDoubleValue();
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
    * Computes the velocity multiplier in the swing phase when using one CMP in each foot. The desired
    * ICP position computed from the stance entry CMP and the next corner point, not the initial ICP location.
    */
   public void computeInSwingOneCMPVelocity(double omega0)
   {
      velocityMultiplier.set(omega0 * positionMultiplier.getDoubleValue());
   }




   /**
    * Computes the position multiplier in the swing phase when using two CMPs in each foot.
    *
    * @param timeInState time in the swing state.
    * @param omega0 natural frequency of the inverted pendulum
    */
   public void computeSwingSegmented(List<YoDouble> singleSupportDurations, double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         computeSwingFirstSegment(singleSupportDurations, timeInState, omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         computeSwingThirdSegment();
      else
         computeSwingSecondSegment(timeInState, omega0);
   }

   /**
    * Computes the position multiplier when in the first segment in the swing phase. During this
    * segment, the ICP plan is on the entry CMP. In this phase, the position is given by <br>
    *    &xi; = e<sup>&omega; t</sup> &xi;<sub>0</sub> + (1.0 - e<sup>&omega; t</sup>) r<sub>cmp,H</sub><br>
    * So the multiplier is simply &gamma;<sub>0</sub> = e<sup>&omega; t</sup>
    *
    * @param timeInState time in the swing state
    * @param omega0 natural frequency of the inverted pendulum.
    */
   public void computeSwingFirstSegment(List<YoDouble> singleSupportDurations, double timeInState, double omega0)
   {
      if (blendFromInitial)
      {
         double projectionMultiplier = Math.exp(omega0 * timeInState);
         double recursionMultiplier = 0.0;

         double blendingTime = blendingFraction * startOfSplineTime.getDoubleValue();
         blendingTime = Math.max(blendingTime, minimumBlendingTime);
         double phaseInState = MathTools.clamp(timeInState / blendingTime, 0.0, 1.0);

         double multiplier = InterpolationTools.linearInterpolate(projectionMultiplier, recursionMultiplier, phaseInState);
         positionMultiplier.set(multiplier);
      }
      else
      {
         positionMultiplier.set(Math.exp(omega0 * timeInState));
      }
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
      swingInitialICPMatrix.compute(omega0);

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

      CommonOps.mult(cubicMatrix, swingInitialICPMatrix, matrixOut);

      positionMultiplier.set(matrixOut.get(0, 0));
   }

   /**
    * Computes the position multiplier in the swing phase when using two CMPs in each foot. The desired
    * ICP position computed from the stance exit CMP and the next corner point, not the initial ICP location.
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
   public void computeSwingSegmentedVelocity(double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         computeSwingFirstSegmentVelocity(omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         computeSwingThirdSegmentVelocity();
      else
         computeSwingSecondSegmentVelocity();
   }

   /**
    * Computes the velocity multiplier in the first segment of the swing phase.
    *
    * @param omega0 natural frequency of the inverted pendulum.
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
      CommonOps.mult(cubicDerivativeMatrix, swingInitialICPMatrix, matrixOut);

      velocityMultiplier.set(matrixOut.get(0, 0));
   }

   /**
    * Computes the position multiplier in the swing phase when using two CMPs in each foot. The desired
    * ICP position computed from the stance exit CMP and the next corner point, not the initial ICP location.
    */
   public void computeSwingThirdSegmentVelocity()
   {
      velocityMultiplier.set(0.0);
   }
}
