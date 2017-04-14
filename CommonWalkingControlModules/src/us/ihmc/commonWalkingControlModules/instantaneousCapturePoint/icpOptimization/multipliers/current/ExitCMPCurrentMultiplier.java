package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.SwingExitCMPMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class ExitCMPCurrentMultiplier
{
   private final CubicMatrix cubicMatrix;
   private final CubicDerivativeMatrix cubicDerivativeMatrix;

   private final boolean givenCubicMatrix;
   private final boolean givenCubicDerivativeMatrix;

   private final SwingExitCMPMatrix swingExitCMPMatrix;

   private final List<DoubleYoVariable> swingSplitFractions;

   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   public ExitCMPCurrentMultiplier( List<DoubleYoVariable> swingSplitFractions, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime,
         YoVariableRegistry registry)
   {
      this(swingSplitFractions, startOfSplineTime, endOfSplineTime, null, null, registry);
   }

   public ExitCMPCurrentMultiplier(List<DoubleYoVariable> swingSplitFractions, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime,
         CubicMatrix cubicMatrix, CubicDerivativeMatrix cubicDerivativeMatrix, YoVariableRegistry registry)
   {
      positionMultiplier = new DoubleYoVariable("ExitCMPCurrentMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable("ExitCMPCurrentVelocityMultiplier", registry);

      this.swingSplitFractions = swingSplitFractions;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;

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

      swingExitCMPMatrix = new SwingExitCMPMatrix(swingSplitFractions, endOfSplineTime);
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

   public void compute(ArrayList<DoubleYoVariable> singleSupportDurations, double timeInState, boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      double positionMultiplier, velocityMultiplier;
      if (isInTransfer)
      {
         positionMultiplier = computeInTransfer();
      }
      else
      {
         if (useTwoCMPs)
            positionMultiplier = computeSegmentedSwing(singleSupportDurations, timeInState, omega0);
         else
            positionMultiplier = computeInSwingOneCMP();
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
            velocityMultiplier = computeInSwingOneCMPVelocity();
      }

      this.velocityMultiplier.set(velocityMultiplier);
   }

   private double computeInTransfer()
   {
      return 0.0;
   }

   private double computeInSwingOneCMP()
   {
      return 0.0;
   }

   private double computeInTransferVelocity()
   {
      return 0.0;
   }

   private double computeInSwingOneCMPVelocity()
   {
      return 0.0;
   }

   private double computeSegmentedSwing(ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeSwingFirstSegment();
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeSwingThirdSegment(singleSupportDurations, timeInState, omega0);
      else
         return computeSwingSecondSegment(singleSupportDurations, timeInState, omega0);
   }





   private double computeSwingFirstSegment()
   {
      return 0.0;
   }

   private double computeSwingSecondSegment(ArrayList<DoubleYoVariable> singleSupportDurations, double timeInState, double omega0)
   {
      swingExitCMPMatrix.compute(singleSupportDurations, omega0);

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

      return matrixOut.get(0, 0);
   }

   private double computeSwingThirdSegment(ArrayList<DoubleYoVariable> singleSupportDurations, double timeInState, double omega0)
   {
      double currentSwingOnEntryCMP = swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();

      double timeInSwingOnExitCMP = timeInState - currentSwingOnEntryCMP;

      return 1.0 - Math.exp(omega0 * timeInSwingOnExitCMP);
   }





   private double computeSegmentedSwingVelocity(double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeSwingFirstSegmentVelocity();
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeSwingThirdSegmentVelocity(omega0);
      else
         return computeSwingSecondSegmentVelocity();
   }

   private double computeSwingFirstSegmentVelocity()
   {
      return 0.0;
   }

   private double computeSwingSecondSegmentVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, swingExitCMPMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeSwingThirdSegmentVelocity(double omega0)
   {
      return omega0 * (positionMultiplier.getDoubleValue() - 1.0);
   }
}
