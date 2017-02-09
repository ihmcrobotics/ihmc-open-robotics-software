package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.SwingInitialICPMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.TransferInitialICPMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class InitialICPCurrentMultiplier
{
   private final CubicMatrix cubicMatrix;
   private final CubicDerivativeMatrix cubicDerivativeMatrix;

   private final TransferInitialICPMatrix transferInitialICPMatrix;
   private final SwingInitialICPMatrix swingInitialICPMatrix;

   private final DoubleYoVariable exitCMPRatio;
   private final DoubleYoVariable upcomingDoubleSupportSplitRatio;
   private final DoubleYoVariable defaultDoubleSupportSplitRatio;

   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;
   private final DoubleYoVariable totalTrajectoryTime;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   private final boolean projectForward;

   public InitialICPCurrentMultiplier(DoubleYoVariable upcomingDoubleSupportSplitRatio, DoubleYoVariable defaultDoubleSupportSplitRatio,
         DoubleYoVariable exitCMPRatio, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, DoubleYoVariable totalTrajectoryTime,
         boolean projectForward, YoVariableRegistry registry)
   {
      positionMultiplier = new DoubleYoVariable("InitialICPCurrentMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable("InitialICPCurrentVelocityMultiplier", registry);

      this.exitCMPRatio = exitCMPRatio;
      this.upcomingDoubleSupportSplitRatio = upcomingDoubleSupportSplitRatio;
      this.defaultDoubleSupportSplitRatio = defaultDoubleSupportSplitRatio;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;
      this.totalTrajectoryTime = totalTrajectoryTime;

      this.projectForward = projectForward;

      cubicMatrix = new CubicMatrix();
      cubicDerivativeMatrix = new CubicDerivativeMatrix();

      transferInitialICPMatrix = new TransferInitialICPMatrix();
      swingInitialICPMatrix = new SwingInitialICPMatrix(startOfSplineTime);
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

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, double timeInState,
         boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      double positionMultiplier, velocityMultiplier;
      if (isInTransfer)
      {
         positionMultiplier = computeInTransfer(doubleSupportDurations, timeInState);
      }
      else
      {
         if (useTwoCMPs)
            positionMultiplier = computeSwingSegmented(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
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
            velocityMultiplier = computeSwingSegmentedVelocity(timeInState, omega0);
         else
            velocityMultiplier = computeInSwingOneCMPVelocity();
      }

      this.velocityMultiplier.set(velocityMultiplier);
   }

   private double computeInTransfer(ArrayList<DoubleYoVariable> doubleSupportDurations, double timeInState)
   {
      transferInitialICPMatrix.compute();

      double splineDuration = doubleSupportDurations.get(0).getDoubleValue();

      cubicDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicDerivativeMatrix.update(timeInState, true);
      cubicMatrix.setSegmentDuration(splineDuration);
      cubicMatrix.update(timeInState, true);

      CommonOps.mult(cubicMatrix, transferInitialICPMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeInTransferVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, transferInitialICPMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }





   private double computeInSwingOneCMP()
   {
      return 0.0;
   }

   private double computeInSwingOneCMPVelocity()
   {
      return 0.0;
   }




   private double computeSwingSegmented(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeSwingFirstSegment(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeSwingThirdSegment();
      else
         return computeSwingSecondSegment(timeInState, omega0);
   }

   private double computeSwingFirstSegment(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeInState, double omega0)
   {
      if (projectForward)
      {
         return Math.exp(omega0 * timeInState);
      }
      else
      {
         double currentDoubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();
         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double stepDuration = currentDoubleSupportDuration + singleSupportDuration;


         double timeSpentOnEntryCMP = (1.0 - exitCMPRatio.getDoubleValue()) * stepDuration;
         double endOfDoubleSupportDuration = (1.0 - defaultDoubleSupportSplitRatio.getDoubleValue()) * currentDoubleSupportDuration;
         double initialSingleSupportDuration = timeSpentOnEntryCMP - endOfDoubleSupportDuration;

         double duration = timeInState - initialSingleSupportDuration;
         double projection = Math.exp(omega0 * duration);

         return projection;
      }
   }

   private double computeSwingSecondSegment(double timeInState, double omega0)
   {
      swingInitialICPMatrix.compute(omega0);

      double timeInSpline = timeInState - startOfSplineTime.getDoubleValue();
      double splineDuration = endOfSplineTime.getDoubleValue() - startOfSplineTime.getDoubleValue();

      cubicDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicDerivativeMatrix.update(timeInSpline, true);
      cubicMatrix.setSegmentDuration(splineDuration);
      cubicMatrix.update(timeInSpline, true);

      CommonOps.mult(cubicMatrix, swingInitialICPMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeSwingThirdSegment()
   {
      return computeInSwingOneCMP();
   }




   private double computeSwingSegmentedVelocity(double timeInState, double omega0)
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
      return omega0 * positionMultiplier.getDoubleValue();
   }

   private double computeSwingSecondSegmentVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, swingInitialICPMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeSwingThirdSegmentVelocity()
   {
      return computeInSwingOneCMPVelocity();
   }
}
