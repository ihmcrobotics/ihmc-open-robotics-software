package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current.*;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.EfficientCubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.EfficientCubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion.RecursionMultipliers;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;

import java.util.ArrayList;
import java.util.List;

public class StateMultiplierCalculator
{
   private final List<DoubleYoVariable> doubleSupportDurations;
   private final List<DoubleYoVariable> singleSupportDurations;
   private final List<DoubleYoVariable> swingSplitFractions;

   private final DoubleYoVariable maximumSplineDuration;
   private final DoubleYoVariable minimumSplineDuration;
   private final DoubleYoVariable minimumTimeToSpendOnExitCMP;
   private final DoubleYoVariable totalTrajectoryTime;
   private final DoubleYoVariable timeSpentOnInitialCMP;
   private final DoubleYoVariable timeSpentOnFinalCMP;
   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;
   private final IntegerYoVariable currentSwingSegment;

   private final RecursionMultipliers recursionMultipliers;

   private final ExitCMPCurrentMultiplier exitCMPCurrentMultiplier;
   private final EntryCMPCurrentMultiplier entryCMPCurrentMultiplier;
   private final InitialICPCurrentMultiplier initialICPCurrentMultiplier;
   private final InitialICPVelocityCurrentMultiplier initialICPVelocityCurrentMultiplier;
   private final StateEndCurrentMultiplier stateEndCurrentMultiplier;

   private final EfficientCubicMatrix cubicMatrix;
   private final EfficientCubicDerivativeMatrix cubicDerivativeMatrix;

   private final int maxNumberOfFootstepsToConsider;

   public StateMultiplierCalculator(CapturePointPlannerParameters icpPlannerParameters, List<DoubleYoVariable> doubleSupportDurations,
         List<DoubleYoVariable> singleSupportDurations, List<DoubleYoVariable> transferSplitFractions,
         List<DoubleYoVariable> swingSplitFractions, int maxNumberOfFootstepsToConsider, String yoNamePrefix, YoVariableRegistry parentRegistry)
   {
      this.maxNumberOfFootstepsToConsider = maxNumberOfFootstepsToConsider;
      this.doubleSupportDurations = doubleSupportDurations;
      this.singleSupportDurations = singleSupportDurations;
      this.swingSplitFractions = swingSplitFractions;

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      maximumSplineDuration = new DoubleYoVariable(yoNamePrefix + "MaximumSplineDuration", registry);
      minimumSplineDuration = new DoubleYoVariable(yoNamePrefix + "MinimumSplineDuration", registry);
      minimumTimeToSpendOnExitCMP = new DoubleYoVariable(yoNamePrefix + "MinimumTimeToSpendOnExitCMP", registry);

      minimumSplineDuration.set(0.1);
      maximumSplineDuration.set(icpPlannerParameters.getMaxDurationForSmoothingEntryToExitCMPSwitch());
      minimumTimeToSpendOnExitCMP.set(icpPlannerParameters.getMinTimeToSpendOnExitCMPInSingleSupport());

      totalTrajectoryTime = new DoubleYoVariable(yoNamePrefix + "TotalTrajectoryTime", registry);
      timeSpentOnInitialCMP = new DoubleYoVariable(yoNamePrefix + "TimeSpentOnInitialCMP", registry);
      timeSpentOnFinalCMP = new DoubleYoVariable(yoNamePrefix + "TimeSpentOnFinalCMP", registry);
      startOfSplineTime = new DoubleYoVariable(yoNamePrefix + "StartOfSplineTime", registry);
      endOfSplineTime = new DoubleYoVariable(yoNamePrefix + "EndOfSplineTime", registry);
      currentSwingSegment = new IntegerYoVariable(yoNamePrefix + "CurrentSegment", registry);


      cubicMatrix = new EfficientCubicMatrix();
      cubicDerivativeMatrix = new EfficientCubicDerivativeMatrix();

      boolean clipTime = true;

      exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, cubicMatrix,
            cubicDerivativeMatrix, yoNamePrefix, clipTime, registry);
      entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime, cubicMatrix, cubicDerivativeMatrix, yoNamePrefix, clipTime, registry);
      initialICPCurrentMultiplier = new InitialICPCurrentMultiplier(startOfSplineTime, endOfSplineTime, cubicMatrix, cubicDerivativeMatrix, yoNamePrefix,
            registry);
      initialICPVelocityCurrentMultiplier = new InitialICPVelocityCurrentMultiplier(cubicMatrix, cubicDerivativeMatrix, yoNamePrefix, registry);
      stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime,
            cubicMatrix, cubicDerivativeMatrix, yoNamePrefix, clipTime, registry);

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
         initialICPCurrentMultiplier.computeInSwingOneCMP();
         initialICPVelocityCurrentMultiplier.computeInSwingOneCMP();
         stateEndCurrentMultiplier.computeInSwingOneCMP(singleSupportDurations, doubleSupportDurations, timeInState, omega0);

         exitCMPCurrentMultiplier.computeInSwingOneCMPVelocity();
         entryCMPCurrentMultiplier.computeInSwingOneCMPVelocity(omega0);
         initialICPCurrentMultiplier.computeInSwingOneCMPVelocity();
         initialICPVelocityCurrentMultiplier.computeInSwingOneCMPVelocity();
         stateEndCurrentMultiplier.computeInSwingOneCMPVelocity(omega0);
      }
      else if (timeInState < startOfSplineTime.getDoubleValue())
      {
         currentSwingSegment.set(1);

         exitCMPCurrentMultiplier.computeSwingFirstSegment();
         entryCMPCurrentMultiplier.computeSwingFirstSegment(timeInState, omega0);
         initialICPCurrentMultiplier.computeSwingFirstSegment(timeInState, omega0);
         initialICPVelocityCurrentMultiplier.computeSwingFirstSegment();
         stateEndCurrentMultiplier.computeSwingFirstSegment();

         exitCMPCurrentMultiplier.computeSwingFirstSegmentVelocity();
         entryCMPCurrentMultiplier.computeSwingFirstSegmentVelocity(omega0);
         initialICPCurrentMultiplier.computeSwingFirstSegmentVelocity(omega0);
         initialICPVelocityCurrentMultiplier.computeSwingFirstSegmentVelocity();
         stateEndCurrentMultiplier.computeSwingFirstSegmentVelocity();
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
         entryCMPCurrentMultiplier.computeSwingSecondSegment(timeInState, omega0);
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

   private final FramePoint2d tmpPoint = new FramePoint2d();
   private final FramePoint2d tmpEntry = new FramePoint2d();
   private final FramePoint2d tmpExit = new FramePoint2d();

   public void reconstructICPCornerPoint(FramePoint2d predictedICPCornerPointToPack, FramePoint2d finalICP, ArrayList<FramePoint2d> footstepLocations,
         ArrayList<FrameVector2d> entryOffsets, ArrayList<FrameVector2d> exitOffsets, int numberOfFootstepsToConsider)
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

   public void yoReconstructICPCornerPoint(FramePoint2d predictedICPCornerPointToPack, FramePoint2d finalICP, ArrayList<YoFramePoint2d> footstepLocations,
         ArrayList<FrameVector2d> entryOffsets, ArrayList<FrameVector2d> exitOffsets, int numberOfFootstepsToConsider)
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

   public void reconstructReferenceICP(FramePoint2d referenceICPToPack, FrameVector2d referenceICPVelocityToPack, FramePoint2d predictedICPCornerPoint,
         FramePoint2d entryCMP, FramePoint2d exitCMP, FramePoint2d initialICP, FrameVector2d initialICPVelocity)
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
