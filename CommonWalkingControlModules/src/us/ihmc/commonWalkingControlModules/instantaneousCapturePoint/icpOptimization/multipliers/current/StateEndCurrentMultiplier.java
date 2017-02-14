package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.SwingStateEndMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.TransferStateEndMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class StateEndCurrentMultiplier
{
   private final CubicMatrix cubicMatrix;
   private final CubicDerivativeMatrix cubicDerivativeMatrix;

   private final boolean givenCubicMatrix;
   private final boolean givenCubicDerivativeMatrix;

   private final TransferStateEndMatrix transferStateEndMatrix;
   private final SwingStateEndMatrix swingStateEndMatrix;

   private final DoubleYoVariable exitCMPRatio;
   private final DoubleYoVariable upcomingDoubleSupportSplitRatio;
   private final DoubleYoVariable defaultDoubleSupportSplitRatio;

   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   private final boolean projectForward;

   public StateEndCurrentMultiplier(DoubleYoVariable upcomingDoubleSupportSplitRatio, DoubleYoVariable defaultDoubleSupportSplitRatio,
         DoubleYoVariable exitCMPRatio, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, boolean projectForward, YoVariableRegistry registry)
   {
      this(upcomingDoubleSupportSplitRatio, defaultDoubleSupportSplitRatio, exitCMPRatio, startOfSplineTime, endOfSplineTime, null, null, projectForward, registry);
   }

   public StateEndCurrentMultiplier(DoubleYoVariable upcomingDoubleSupportSplitRatio, DoubleYoVariable defaultDoubleSupportSplitRatio,
         DoubleYoVariable exitCMPRatio, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, CubicMatrix cubicMatrix,
         CubicDerivativeMatrix cubicDerivativeMatrix, boolean projectForward, YoVariableRegistry registry)
   {
      positionMultiplier = new DoubleYoVariable("StateEndCurrentMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable("StateEndCurrentVelocityMultiplier", registry);

      this.exitCMPRatio = exitCMPRatio;
      this.upcomingDoubleSupportSplitRatio = upcomingDoubleSupportSplitRatio;
      this.defaultDoubleSupportSplitRatio = defaultDoubleSupportSplitRatio;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;

      this.projectForward = projectForward;

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

      transferStateEndMatrix = new TransferStateEndMatrix(defaultDoubleSupportSplitRatio);
      swingStateEndMatrix = new SwingStateEndMatrix(upcomingDoubleSupportSplitRatio, exitCMPRatio, endOfSplineTime);
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
         positionMultiplier = computeInTransfer(doubleSupportDurations, omega0, timeInState);
      }
      else
      {
         if (useTwoCMPs)
            positionMultiplier = computeSwingSegmented(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
         else
            positionMultiplier = computeInSwingOneCMP(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
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
            velocityMultiplier = computeInSwingOneCMPVelocity(omega0);
      }

      this.velocityMultiplier.set(velocityMultiplier);
   }

   private double computeInTransfer(ArrayList<DoubleYoVariable> doubleSupportDurations, double omega0, double timeInState)
   {
      transferStateEndMatrix.compute(doubleSupportDurations, omega0);

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

      return matrixOut.get(0, 0);
   }

   private double computeInTransferVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, transferStateEndMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }





   private double computeInSwingOneCMP(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, double timeInState, double omega0)
   {
      return computeInSwingOneCMP(doubleSupportDurations.get(1).getDoubleValue(), doubleSupportDurations.get(0).getDoubleValue(),
            singleSupportDurations.get(0).getDoubleValue(), timeInState, omega0);
   }

   private double computeInSwingOneCMP(double upcomingDoubleSupportDuration, double doubleSupportDuration, double singleSupportDuration, double timeInState, double omega0)
   {
      double stepDuration = doubleSupportDuration + singleSupportDuration;
      double timeSpentOnExitCMP = exitCMPRatio.getDoubleValue() * stepDuration;
      double upcomingInitialDoubleSupportDuration = upcomingDoubleSupportSplitRatio.getDoubleValue() * upcomingDoubleSupportDuration;

      double duration = timeInState - singleSupportDuration + timeSpentOnExitCMP - upcomingInitialDoubleSupportDuration;
      return Math.exp(omega0 * duration);
   }


   private double computeInSwingOneCMPVelocity(double omega0)
   {
      return omega0 * positionMultiplier.getDoubleValue();
   }




   private double computeSwingSegmented(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeSwingFirstSegment(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeSwingThirdSegment(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
      else
         return computeSwingSecondSegment(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
   }

   private double computeSwingFirstSegment(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeInState, double omega0)
   {
      if (projectForward)
      {
         return 0.0;
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

   private double computeSwingSecondSegment(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeInState, double omega0)
   {
      swingStateEndMatrix.compute(doubleSupportDurations, singleSupportDurations, omega0);

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

      return matrixOut.get(0, 0);
   }

   private double computeSwingThirdSegment(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, double timeInState, double omega0)
   {
      return computeInSwingOneCMP(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
   }




   private double computeSwingSegmentedVelocity(double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeSwingFirstSegmentVelocity(omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeSwingThirdSegmentVelocity(omega0);
      else
         return computeSwingSecondSegmentVelocity();
   }

   private double computeSwingFirstSegmentVelocity(double omega0)
   {
      return omega0 * positionMultiplier.getDoubleValue();
   }

   private double computeSwingSecondSegmentVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, swingStateEndMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeSwingThirdSegmentVelocity(double omega0)
   {
      return omega0 * positionMultiplier.getDoubleValue();
   }
}
