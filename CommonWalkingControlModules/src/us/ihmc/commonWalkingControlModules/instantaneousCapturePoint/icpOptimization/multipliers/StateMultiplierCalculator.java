package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers;

import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current.*;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.EfficientCubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.EfficientCubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion.RecursionMultipliers;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.math.frames.YoFramePoint2d;

import java.util.ArrayList;
import java.util.List;

public class StateMultiplierCalculator
{
   private final List<YoDouble> doubleSupportDurations;
   private final List<YoDouble> singleSupportDurations;
   private final List<YoDouble> swingSplitFractions;

   private final YoDouble maximumSplineDuration;
   private final YoDouble minimumSplineDuration;
   private final YoDouble minimumTimeToSpendOnExitCMP;
   private final YoDouble totalTrajectoryTime;
   private final YoDouble timeSpentOnInitialCMP;
   private final YoDouble timeSpentOnFinalCMP;
   private final YoDouble startOfSplineTime;
   private final YoDouble endOfSplineTime;
   private final YoInteger currentSwingSegment;

   private final RecursionMultipliers recursionMultipliers;

   private final ExitCMPCurrentMultiplier exitCMPCurrentMultiplier;
   private final EntryCMPCurrentMultiplier entryCMPCurrentMultiplier;
   private final InitialICPCurrentMultiplier initialICPCurrentMultiplier;
   private final InitialICPVelocityCurrentMultiplier initialICPVelocityCurrentMultiplier;
   private final StateEndCurrentMultiplier stateEndCurrentMultiplier;

   private final EfficientCubicMatrix cubicMatrix;
   private final EfficientCubicDerivativeMatrix cubicDerivativeMatrix;

   private final int maxNumberOfFootstepsToConsider;

   private static final boolean blendFromInitial = true;
   private static final double blendingFraction = 0.5;
   private static final double minimumBlendingTime = 0.05;

   public StateMultiplierCalculator(ICPTrajectoryPlannerParameters icpPlannerParameters, List<YoDouble> doubleSupportDurations,
                                    List<YoDouble> singleSupportDurations, List<YoDouble> transferSplitFractions,
                                    List<YoDouble> swingSplitFractions, int maxNumberOfFootstepsToConsider, String yoNamePrefix, YoVariableRegistry parentRegistry)
   {
      this.maxNumberOfFootstepsToConsider = maxNumberOfFootstepsToConsider;
      this.doubleSupportDurations = doubleSupportDurations;
      this.singleSupportDurations = singleSupportDurations;
      this.swingSplitFractions = swingSplitFractions;

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      maximumSplineDuration = new YoDouble(yoNamePrefix + "MaximumSplineDuration", registry);
      minimumSplineDuration = new YoDouble(yoNamePrefix + "MinimumSplineDuration", registry);
      minimumTimeToSpendOnExitCMP = new YoDouble(yoNamePrefix + "MinimumTimeToSpendOnExitCMP", registry);

      minimumSplineDuration.set(0.1);
      maximumSplineDuration.set(icpPlannerParameters.getMaxDurationForSmoothingEntryToExitCoPSwitch());
      minimumTimeToSpendOnExitCMP.set(icpPlannerParameters.getMinTimeToSpendOnExitCoPInSingleSupport());

      totalTrajectoryTime = new YoDouble(yoNamePrefix + "TotalTrajectoryTime", registry);
      timeSpentOnInitialCMP = new YoDouble(yoNamePrefix + "TimeSpentOnInitialCMP", registry);
      timeSpentOnFinalCMP = new YoDouble(yoNamePrefix + "TimeSpentOnFinalCMP", registry);
      startOfSplineTime = new YoDouble(yoNamePrefix + "StartOfSplineTime", registry);
      endOfSplineTime = new YoDouble(yoNamePrefix + "EndOfSplineTime", registry);
      currentSwingSegment = new YoInteger(yoNamePrefix + "CurrentSegment", registry);


      cubicMatrix = new EfficientCubicMatrix();
      cubicDerivativeMatrix = new EfficientCubicDerivativeMatrix();

      boolean clipTime = true;

      exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, cubicMatrix,
            cubicDerivativeMatrix, yoNamePrefix, clipTime, blendFromInitial, blendingFraction, minimumBlendingTime, registry);
      entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime, cubicMatrix, cubicDerivativeMatrix, yoNamePrefix, clipTime, blendFromInitial, blendingFraction, minimumBlendingTime, registry);
      initialICPCurrentMultiplier = new InitialICPCurrentMultiplier(startOfSplineTime, endOfSplineTime, cubicMatrix, cubicDerivativeMatrix, blendFromInitial,
            blendingFraction, minimumBlendingTime, yoNamePrefix, registry);
      initialICPVelocityCurrentMultiplier = new InitialICPVelocityCurrentMultiplier(cubicMatrix, cubicDerivativeMatrix, yoNamePrefix, registry);
      stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime,
            cubicMatrix, cubicDerivativeMatrix, yoNamePrefix, clipTime, blendFromInitial, blendingFraction, minimumBlendingTime, registry);

      recursionMultipliers = new RecursionMultipliers(yoNamePrefix, maxNumberOfFootstepsToConsider, swingSplitFractions, transferSplitFractions,
            registry);

      parentRegistry.addChild(registry);
   }

   public void computeRecursionMultipliers(int numberOfStepsToConsider, int numberOfStepsRegistered, boolean useTwoCMPs, double omega0)
   {
      recursionMultipliers.reset();

      if (numberOfStepsToConsider > maxNumberOfFootstepsToConsider)
         throw new RuntimeException("Requesting too many steps.");

      recursionMultipliers.compute(numberOfStepsToConsider, numberOfStepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega0);
   }

   public double getFinalICPRecursionMultiplier()
   {
      return recursionMultipliers.getFinalICPMultiplier();
   }

   public double getExitCMPRecursionMultiplier(int footstepIndex)
   {
      return recursionMultipliers.getExitMultiplier(footstepIndex);
   }

   public double getEntryCMPRecursionMultiplier(int footstepIndex)
   {
      return recursionMultipliers.getEntryMultiplier(footstepIndex);
   }

   public double getFootstepRecursionMultiplier(boolean useTwoCMPs, int footstepIndex)
   {
      double footstepRecursionMultiplier;
      if (useTwoCMPs)
      {
         double entryMutliplier = getEntryCMPRecursionMultiplier(footstepIndex);
         double exitMutliplier = getExitCMPRecursionMultiplier(footstepIndex);

         footstepRecursionMultiplier = entryMutliplier + exitMutliplier;
      }
      else
      {
         footstepRecursionMultiplier = getEntryCMPRecursionMultiplier(footstepIndex);
      }
      footstepRecursionMultiplier *= getStateEndCurrentMultiplier();

      return footstepRecursionMultiplier;
   }


   public void resetCurrentMultipliers()
   {
      exitCMPCurrentMultiplier.reset();
      entryCMPCurrentMultiplier.reset();
      initialICPCurrentMultiplier.reset();
      initialICPVelocityCurrentMultiplier.reset();
      stateEndCurrentMultiplier.reset();
   }

   public void initializeForDoubleSupport()
   {
      currentSwingSegment.set(0);

      timeSpentOnInitialCMP.setToNaN();
      timeSpentOnFinalCMP.setToNaN();
      totalTrajectoryTime.setToNaN();
      startOfSplineTime.setToNaN();
      endOfSplineTime.setToNaN();

      double doubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();
      cubicDerivativeMatrix.setSegmentDuration(doubleSupportDuration);
      cubicMatrix.setSegmentDuration(doubleSupportDuration);
   }

   public void initializeForSingleSupport()
   {
      double timeOnEntryDuringSwing = swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();
      double timeOnExitDuringSwing = (1.0 - swingSplitFractions.get(0).getDoubleValue()) * singleSupportDurations.get(0).getDoubleValue();

      timeSpentOnInitialCMP.set(timeOnEntryDuringSwing);
      timeSpentOnFinalCMP.set(timeOnExitDuringSwing);
      totalTrajectoryTime.set(timeOnEntryDuringSwing + timeOnExitDuringSwing);

      double alpha = 0.50;
      double minTimeOnExitCMP = minimumTimeToSpendOnExitCMP.getDoubleValue();
      minTimeOnExitCMP = Math.min(minTimeOnExitCMP, timeSpentOnFinalCMP.getDoubleValue() - alpha * minimumSplineDuration.getDoubleValue());

      double startOfSplineTime = timeSpentOnInitialCMP.getDoubleValue() - alpha * maximumSplineDuration.getDoubleValue();
      startOfSplineTime = Math.max(startOfSplineTime, 0.0);
      this.startOfSplineTime.set(startOfSplineTime);

      double endOfSplineTime = timeSpentOnInitialCMP.getDoubleValue() + (1.0 - alpha) * maximumSplineDuration.getDoubleValue();
      endOfSplineTime = Math.min(endOfSplineTime, totalTrajectoryTime.getDoubleValue() - minTimeOnExitCMP);
      if (endOfSplineTime > totalTrajectoryTime.getDoubleValue() - minTimeOnExitCMP)
      {
         endOfSplineTime = totalTrajectoryTime.getDoubleValue() - minTimeOnExitCMP;
         startOfSplineTime = timeSpentOnInitialCMP.getDoubleValue() - (endOfSplineTime - timeSpentOnInitialCMP.getDoubleValue());
      }
      this.startOfSplineTime.set(startOfSplineTime);
      this.endOfSplineTime.set(endOfSplineTime);

      double splineDuration = endOfSplineTime - startOfSplineTime;
      cubicMatrix.setSegmentDuration(splineDuration);
      cubicDerivativeMatrix.setSegmentDuration(splineDuration);
   }

   public void computeCurrentMultipliers(int numberOfFootstepsToConsider, double timeInState, boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      resetCurrentMultipliers();

      if (isInTransfer)
      {
         cubicMatrix.update(timeInState);
         cubicDerivativeMatrix.update(timeInState);

         exitCMPCurrentMultiplier.computeInTransfer(numberOfFootstepsToConsider, singleSupportDurations, doubleSupportDurations, timeInState, useTwoCMPs, omega0);
         entryCMPCurrentMultiplier.computeInTransfer(numberOfFootstepsToConsider, singleSupportDurations, doubleSupportDurations, timeInState, useTwoCMPs, omega0);
         initialICPCurrentMultiplier.computeInTransfer(doubleSupportDurations, timeInState);
         initialICPVelocityCurrentMultiplier.computeInTransfer(doubleSupportDurations, timeInState);
         stateEndCurrentMultiplier.computeInTransfer(numberOfFootstepsToConsider, singleSupportDurations, doubleSupportDurations, timeInState, useTwoCMPs, omega0);

         exitCMPCurrentMultiplier.computeInTransferVelocity();
         entryCMPCurrentMultiplier.computeInTransferVelocity();
         initialICPCurrentMultiplier.computeInTransferVelocity();
         initialICPVelocityCurrentMultiplier.computeInTransferVelocity();
         stateEndCurrentMultiplier.computeInTransferVelocity();
      }
      else if (!useTwoCMPs)
      {
         exitCMPCurrentMultiplier.computeInSwingOneCMP();
         entryCMPCurrentMultiplier.computeInSwingOneCMP(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
         initialICPCurrentMultiplier.computeInSwingOneCMP(singleSupportDurations, timeInState, omega0);
         initialICPVelocityCurrentMultiplier.computeInSwingOneCMP();
         stateEndCurrentMultiplier.computeInSwingOneCMP(singleSupportDurations, doubleSupportDurations, timeInState, omega0);

         exitCMPCurrentMultiplier.computeInSwingOneCMPVelocity();
         entryCMPCurrentMultiplier.computeInSwingOneCMPVelocity(omega0);
         initialICPCurrentMultiplier.computeInSwingOneCMPVelocity(omega0);
         initialICPVelocityCurrentMultiplier.computeInSwingOneCMPVelocity();
         stateEndCurrentMultiplier.computeInSwingOneCMPVelocity(omega0);
      }
      else if (timeInState < startOfSplineTime.getDoubleValue())
      {
         currentSwingSegment.set(1);

         exitCMPCurrentMultiplier.computeSwingFirstSegment(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
         entryCMPCurrentMultiplier.computeSwingFirstSegment(singleSupportDurations, timeInState, omega0);
         initialICPCurrentMultiplier.computeSwingFirstSegment(singleSupportDurations, timeInState, omega0);
         initialICPVelocityCurrentMultiplier.computeSwingFirstSegment();
         stateEndCurrentMultiplier.computeSwingFirstSegment(singleSupportDurations, doubleSupportDurations, timeInState, omega0);

         exitCMPCurrentMultiplier.computeSwingFirstSegmentVelocity(omega0);
         entryCMPCurrentMultiplier.computeSwingFirstSegmentVelocity(omega0);
         initialICPCurrentMultiplier.computeSwingFirstSegmentVelocity(omega0);
         initialICPVelocityCurrentMultiplier.computeSwingFirstSegmentVelocity();
         stateEndCurrentMultiplier.computeSwingFirstSegmentVelocity(omega0);
      }
      else if (timeInState >= endOfSplineTime.getDoubleValue())
      {
         currentSwingSegment.set(3);

         exitCMPCurrentMultiplier.computeSwingThirdSegment(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
         entryCMPCurrentMultiplier.computeSwingThirdSegment();
         initialICPCurrentMultiplier.computeSwingThirdSegment();
         initialICPVelocityCurrentMultiplier.computeSwingThirdSegment();
         stateEndCurrentMultiplier.computeSwingThirdSegment(singleSupportDurations, doubleSupportDurations, timeInState, omega0);

         exitCMPCurrentMultiplier.computeSwingThirdSegmentVelocity(omega0);
         entryCMPCurrentMultiplier.computeSwingThirdSegmentVelocity();
         initialICPCurrentMultiplier.computeSwingThirdSegmentVelocity();
         initialICPVelocityCurrentMultiplier.computeSwingThirdSegmentVelocity();
         stateEndCurrentMultiplier.computeSwingThirdSegmentVelocity(omega0);
      }
      else
      {
         currentSwingSegment.set(2);
         double timeInSpline = timeInState - startOfSplineTime.getDoubleValue();

         cubicMatrix.update(timeInSpline);
         cubicDerivativeMatrix.update(timeInSpline);

         exitCMPCurrentMultiplier.computeSwingSecondSegment(singleSupportDurations, doubleSupportDurations, timeInState, omega0);
         entryCMPCurrentMultiplier.computeSwingSecondSegment(singleSupportDurations, timeInState, omega0);
         initialICPCurrentMultiplier.computeSwingSecondSegment(timeInState, omega0);
         initialICPVelocityCurrentMultiplier.computeSwingSecondSegment();
         stateEndCurrentMultiplier.computeSwingSecondSegment(singleSupportDurations, doubleSupportDurations, timeInState, omega0);

         exitCMPCurrentMultiplier.computeSwingSecondSegmentVelocity();
         entryCMPCurrentMultiplier.computeSwingSecondSegmentVelocity();
         initialICPCurrentMultiplier.computeSwingSecondSegmentVelocity();
         initialICPVelocityCurrentMultiplier.computeSwingSecondSegmentVelocity();
         stateEndCurrentMultiplier.computeSwingSecondSegmentVelocity();
      }
   }

   public double getExitCMPCurrentMultiplier()
   {
      return exitCMPCurrentMultiplier.getPositionMultiplier();
   }

   public double getExitCMPCurrentVelocityMultiplier()
   {
      return exitCMPCurrentMultiplier.getVelocityMultiplier();
   }

   public double getEntryCMPCurrentMultiplier()
   {
      return entryCMPCurrentMultiplier.getPositionMultiplier();
   }

   public double getEntryCMPCurrentVelocityMultiplier()
   {
      return entryCMPCurrentMultiplier.getVelocityMultiplier();
   }

   public double getInitialICPCurrentMultiplier()
   {
      return initialICPCurrentMultiplier.getPositionMultiplier();
   }

   public double getInitialICPCurrentVelocityMultiplier()
   {
      return initialICPCurrentMultiplier.getVelocityMultiplier();
   }

   public double getInitialICPVelocityCurrentMultiplier()
   {
      return initialICPVelocityCurrentMultiplier.getPositionMultiplier();
   }

   public double getInitialICPVelocityCurrentVelocityMultiplier()
   {
      return initialICPVelocityCurrentMultiplier.getVelocityMultiplier();
   }

   public double getStateEndCurrentMultiplier()
   {
      return stateEndCurrentMultiplier.getPositionMultiplier();
   }

   public double getStateEndCurrentVelocityMultiplier()
   {
      return stateEndCurrentMultiplier.getVelocityMultiplier();
   }

   private final FramePoint2D tmpPoint = new FramePoint2D();
   private final FramePoint2D tmpEntry = new FramePoint2D();
   private final FramePoint2D tmpExit = new FramePoint2D();

   public void reconstructICPCornerPoint(FramePoint2D predictedICPCornerPointToPack, FramePoint2D finalICP, ArrayList<FramePoint2D> footstepLocations,
         ArrayList<FrameVector2D> entryOffsets, ArrayList<FrameVector2D> exitOffsets, int numberOfFootstepsToConsider)
   {
      predictedICPCornerPointToPack.set(finalICP);
      predictedICPCornerPointToPack.scale(getFinalICPRecursionMultiplier());

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         tmpEntry.set(footstepLocations.get(i));
         tmpEntry.add(entryOffsets.get(i));
         tmpEntry.scale(getEntryCMPRecursionMultiplier(i));
         predictedICPCornerPointToPack.add(tmpEntry);
      }

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         if (!exitOffsets.get(i).containsNaN())
         {
            tmpExit.set(footstepLocations.get(i));
            tmpExit.add(exitOffsets.get(i));
            tmpExit.scale(getExitCMPRecursionMultiplier(i));
            predictedICPCornerPointToPack.add(tmpExit);
         }
      }
   }

   public void yoReconstructICPCornerPoint(FramePoint2D predictedICPCornerPointToPack, FramePoint2D finalICP, ArrayList<YoFramePoint2d> footstepLocations,
         ArrayList<FrameVector2D> entryOffsets, ArrayList<FrameVector2D> exitOffsets, int numberOfFootstepsToConsider)
   {
      predictedICPCornerPointToPack.set(finalICP);
      predictedICPCornerPointToPack.scale(getFinalICPRecursionMultiplier());

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         tmpEntry.set(footstepLocations.get(i).getFrameTuple2d());
         tmpEntry.add(entryOffsets.get(i));
         tmpEntry.scale(getEntryCMPRecursionMultiplier(i));
         predictedICPCornerPointToPack.add(tmpEntry);
      }

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         tmpExit.set(footstepLocations.get(i).getFrameTuple2d());
         tmpExit.add(exitOffsets.get(i));
         tmpExit.scale(getExitCMPRecursionMultiplier(i));
         predictedICPCornerPointToPack.add(tmpExit);
      }
   }

   public void reconstructReferenceICP(FramePoint2D referenceICPToPack, FrameVector2D referenceICPVelocityToPack, FramePoint2D predictedICPCornerPoint,
         FramePoint2D entryCMP, FramePoint2D exitCMP, FramePoint2D initialICP, FrameVector2D initialICPVelocity)
   {
      referenceICPToPack.set(predictedICPCornerPoint);
      referenceICPToPack.scale(getStateEndCurrentMultiplier());

      referenceICPVelocityToPack.set(predictedICPCornerPoint);
      referenceICPVelocityToPack.scale(getStateEndCurrentVelocityMultiplier());

      tmpPoint.setToZero();
      tmpPoint.set(entryCMP);
      tmpPoint.scale(getEntryCMPCurrentMultiplier());
      referenceICPToPack.add(tmpPoint);

      tmpPoint.setToZero();
      tmpPoint.set(entryCMP);
      tmpPoint.scale(getEntryCMPCurrentVelocityMultiplier());
      referenceICPVelocityToPack.add(tmpPoint);

      if (!exitCMP.containsNaN())
      {
         tmpPoint.setToZero();
         tmpPoint.set(exitCMP);
         tmpPoint.scale(getExitCMPCurrentMultiplier());
         referenceICPToPack.add(tmpPoint);

         tmpPoint.setToZero();
         tmpPoint.set(exitCMP);
         tmpPoint.scale(getExitCMPCurrentVelocityMultiplier());
         referenceICPVelocityToPack.add(tmpPoint);
      }

      tmpPoint.setToZero();
      tmpPoint.set(initialICP);
      tmpPoint.scale(getInitialICPCurrentMultiplier());
      referenceICPToPack.add(tmpPoint);

      tmpPoint.setToZero();
      tmpPoint.set(initialICP);
      tmpPoint.scale(getInitialICPCurrentVelocityMultiplier());
      referenceICPVelocityToPack.add(tmpPoint);

      tmpPoint.setToZero();
      tmpPoint.set(initialICPVelocity);
      tmpPoint.scale(getInitialICPVelocityCurrentMultiplier());
      referenceICPToPack.add(tmpPoint);

      tmpPoint.setToZero();
      tmpPoint.set(initialICPVelocity);
      tmpPoint.scale(getInitialICPVelocityCurrentVelocityMultiplier());
      referenceICPVelocityToPack.add(tmpPoint);
   }
}
