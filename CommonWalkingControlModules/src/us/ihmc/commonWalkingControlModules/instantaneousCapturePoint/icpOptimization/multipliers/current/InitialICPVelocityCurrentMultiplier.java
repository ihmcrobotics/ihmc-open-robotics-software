package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.EfficientCubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.EfficientCubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.TransferInitialICPVelocityMatrix;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

/**
 * Multiplier of the initial ICP velocity value at the beginning of the current state. Used to compute the desired current
 * ICP location from the recursively found end-of-state ICP corner point
 */
public class InitialICPVelocityCurrentMultiplier
{
   /** Cubic spline matrix that multiplies the boundary conditions to compute the current value. */
   private final EfficientCubicMatrix cubicMatrix;
   /** Cubic spline matrix that multiplies the boundary conditions to compute the current derivative value. */
   private final EfficientCubicDerivativeMatrix cubicDerivativeMatrix;

   /** whether or not the cubic matrix needs to be updated inside this class or is updated outside it. */
   private final boolean givenCubicMatrix;
   /** whether or not the cubic derivative matrix needs to be updated inside this class or is updated outside it. */
   private final boolean givenCubicDerivativeMatrix;

   /** Boundary conditions matrix for the initial ICP velocity when in transfer. */
   private final TransferInitialICPVelocityMatrix transferInitialICPVelocityMatrix;

   /** data holder for multiplied values */
   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   /** multiplier of the initial ICP to compute the current ICP location. */
   private final YoDouble positionMultiplier;
   /** multiplier of the initial ICP to compute the current ICP velocity. */
   private final YoDouble velocityMultiplier;

   public InitialICPVelocityCurrentMultiplier(String yoNamePrefix, YoVariableRegistry registry)
   {
      this(null, null, yoNamePrefix, registry);
   }

   public InitialICPVelocityCurrentMultiplier(EfficientCubicMatrix cubicMatrix, EfficientCubicDerivativeMatrix cubicDerivativeMatrix, String yoNamePrefix,
         YoVariableRegistry registry)
   {
      positionMultiplier = new YoDouble(yoNamePrefix + "InitialICPVelocityCurrentMultiplier", registry);
      velocityMultiplier = new YoDouble(yoNamePrefix + "InitialICPCVelocityCurrentVelocityMultiplier", registry);

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

      transferInitialICPVelocityMatrix = new TransferInitialICPVelocityMatrix();
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
    * Gets the value to multiply the initial ICP velocity location by to compute the current ICP location.
    *
    * @return position multiplier.
    */
   public double getPositionMultiplier()
   {
      return positionMultiplier.getDoubleValue();
   }

   /**
    * Gets the value to multiply the initial ICP velocity location by to compute the current ICP velocity.
    *
    * @return velocity multiplier.
    */
   public double getVelocityMultiplier()
   {
      return velocityMultiplier.getDoubleValue();
   }

   /**
    * Computes the initial ICP velocity multiplier. Must be called every control tick.
    *
    * @param doubleSupportDurations vector of double support durations.
    * @param timeInState time in the current state.
    * @param isInTransfer whether or not the robot is currently in the transfer phase.
    */
   public void compute(List<YoDouble> doubleSupportDurations, double timeInState, boolean isInTransfer)
   {
      if (isInTransfer)
         computeInTransfer(doubleSupportDurations, timeInState);
      else
         positionMultiplier.set(0.0);

      if (isInTransfer)
         computeInTransferVelocity();
      else
         velocityMultiplier.set(0.0);
   }

   /**
    * Computes the position multiplier when in the transfer phase. During this phase, the trajectory is a cubic spline,
    * so this is used to calculate the position multiplier. The initial ICP velocity directly sets one of the spline
    * boundary conditions.
    *
    * @param doubleSupportDurations vector of double support durations
    * @param timeInState time in the transfer state
    */
   public void computeInTransfer(List<YoDouble> doubleSupportDurations, double timeInState)
   {
      transferInitialICPVelocityMatrix.compute();

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

      CommonOps.mult(cubicMatrix, transferInitialICPVelocityMatrix, matrixOut);

      positionMultiplier.set(matrixOut.get(0, 0));
   }

   /**
    * Computes the position multiplier when in the transfer phase. During this phase, the trajectory is a
    * cubic spline, so this is used to calculate the position multiplier.
    */
   public void computeInTransferVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, transferInitialICPVelocityMatrix, matrixOut);

      velocityMultiplier.set(matrixOut.get(0, 0));
   }

   public void computeInSwingOneCMP()
   {
      positionMultiplier.set(0.0);
   }

   public void computeInSwingOneCMPVelocity()
   {
      velocityMultiplier.set(0.0);
   }

   public void computeSwingFirstSegment()
   {
      positionMultiplier.set(0.0);
   }

   public void computeSwingSecondSegment()
   {
      positionMultiplier.set(0.0);
   }

   public void computeSwingThirdSegment()
   {
      positionMultiplier.set(0.0);
   }

   public void computeSwingFirstSegmentVelocity()
   {
      velocityMultiplier.set(0.0);
   }

   public void computeSwingSecondSegmentVelocity()
   {
      velocityMultiplier.set(0.0);
   }

   public void computeSwingThirdSegmentVelocity()
   {
      velocityMultiplier.set(0.0);
   }
}
