package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion.*;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class NewStateMultiplierCalculator
{
   private static final String namePrefix = "controller";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

   private final DoubleYoVariable defaultDoubleSupportSplitFraction;
   private final DoubleYoVariable upcomingDoubleSupportSplitFraction;
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   private final DoubleYoVariable maximumSplineDuration;
   private final DoubleYoVariable minimumSplineDuration;
   private final DoubleYoVariable minimumTimeToSpendOnExitCMP;
   private final DoubleYoVariable totalTrajectoryTime;
   private final DoubleYoVariable timeSpentOnInitialCMP;
   private final DoubleYoVariable timeSpentOnFinalCMP;
   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;

   private final NewFinalICPRecursionMultiplier finalICPRecursionMultiplier;
   private final NewStanceExitCMPRecursionMultiplier stanceExitCMPRecursionMultiplier;
   private final NewStanceEntryCMPRecursionMultiplier stanceEntryCMPRecursionMultiplier;
   private final NewExitCMPRecursionMultiplier exitCMPRecursionMultiplier;
   private final NewEntryCMPRecursionMultiplier entryCMPRecursionMultiplier;

   /*
   private final NewFinalICPCurrentMultiplier finalICPCurrentMultiplier;
   private final NewExitCMPCurrentMultiplier exitCMPCurrentMultiplier;
   private final NewEntryCMPCurrentMultiplier entryCMPCurrentMultiplier;
   private final NewInitialICPCurrentMultiplier initialICPCurrentMultiplier;
   private final NewInitialICPVelocityCurrentMultiplier initialICPVelocityCurrentMultiplier;
   */

   private final int maxNumberOfFootstepsToConsider;

   public NewStateMultiplierCalculator(CapturePointPlannerParameters icpPlannerParameters, DoubleYoVariable exitCMPDurationInPercentOfStepTime,
         DoubleYoVariable doubleSupportSplitFraction, int maxNumberOfFootstepsToConsider, YoVariableRegistry parentRegistry)
   {
      this.maxNumberOfFootstepsToConsider = maxNumberOfFootstepsToConsider;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;
      this.defaultDoubleSupportSplitFraction = doubleSupportSplitFraction;
      this.upcomingDoubleSupportSplitFraction = doubleSupportSplitFraction;

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("recursionCalculatorDoubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("recursionCalculatorSingleSupportDuration" + i, registry));
      }

      maximumSplineDuration = new DoubleYoVariable(namePrefix + "MaximumSplineDuration", registry);
      minimumSplineDuration = new DoubleYoVariable(namePrefix + "MinimumSplineDuration", registry);
      minimumTimeToSpendOnExitCMP = new DoubleYoVariable(namePrefix + "MinimumTimeToSpendOnExitCMP", registry);

      minimumSplineDuration.set(0.1);
      maximumSplineDuration.set(icpPlannerParameters.getMaxDurationForSmoothingEntryToExitCMPSwitch());
      minimumTimeToSpendOnExitCMP.set(icpPlannerParameters.getMinTimeToSpendOnExitCMPInSingleSupport());

      totalTrajectoryTime = new DoubleYoVariable(namePrefix + "TotalTrajectoryTime", registry);
      timeSpentOnInitialCMP = new DoubleYoVariable(namePrefix + "TimeSpentOnInitialCMP", registry);
      timeSpentOnFinalCMP = new DoubleYoVariable(namePrefix + "TimeSpentOnFinalCMP", registry);
      startOfSplineTime = new DoubleYoVariable(namePrefix + "StartOfSplineTime", registry);
      endOfSplineTime = new DoubleYoVariable(namePrefix + "EndOfSplineTime", registry);

      finalICPRecursionMultiplier = new NewFinalICPRecursionMultiplier(namePrefix, exitCMPDurationInPercentOfStepTime, registry);
      stanceExitCMPRecursionMultiplier = new NewStanceExitCMPRecursionMultiplier(namePrefix, exitCMPDurationInPercentOfStepTime, registry);
      stanceEntryCMPRecursionMultiplier = new NewStanceEntryCMPRecursionMultiplier(namePrefix, exitCMPDurationInPercentOfStepTime, registry);
      exitCMPRecursionMultiplier = new NewExitCMPRecursionMultiplier(namePrefix, maxNumberOfFootstepsToConsider, exitCMPDurationInPercentOfStepTime, registry);
      entryCMPRecursionMultiplier = new NewEntryCMPRecursionMultiplier(namePrefix, maxNumberOfFootstepsToConsider, exitCMPDurationInPercentOfStepTime, registry);

      parentRegistry.addChild(registry);
   }

   public void resetTimes()
   {
      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         doubleSupportDurations.get(i).set(0.0);
         singleSupportDurations.get(i).set(0.0);
      }
   }

   public void submitTimes(int footstepIndex, double doubleSupportDuration, double singleSupportDuration)
   {
      doubleSupportDurations.get(footstepIndex).set(doubleSupportDuration);
      singleSupportDurations.get(footstepIndex).set(singleSupportDuration);
   }

   public void resetRecursionMultipliers()
   {
      finalICPRecursionMultiplier.reset();
      stanceExitCMPRecursionMultiplier.reset();
      stanceEntryCMPRecursionMultiplier.reset();
      exitCMPRecursionMultiplier.reset();
      entryCMPRecursionMultiplier.reset();
   }

   public void computeRecursionMultipliers(int numberOfStepsToConsider, boolean isInTransfer, boolean useTwoCMPs, double omega0)
   {
      resetRecursionMultipliers();

      if (numberOfStepsToConsider > maxNumberOfFootstepsToConsider)
         throw new RuntimeException("Requesting too many steps.");

      finalICPRecursionMultiplier.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0);
      stanceExitCMPRecursionMultiplier.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0);
      stanceEntryCMPRecursionMultiplier.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0);
      exitCMPRecursionMultiplier.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0);
      entryCMPRecursionMultiplier.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0);
   }

   public double getFinalICPRecursionMultiplier()
   {
      return finalICPRecursionMultiplier.getDoubleValue();
   }

   public double getStanceExitCMPRecursionMultiplier()
   {
      return stanceExitCMPRecursionMultiplier.getExitMultiplier();
   }

   public double getStanceEntryCMPRecursionMultiplier()
   {
      return stanceEntryCMPRecursionMultiplier.getEntryMultiplier();
   }

   public double getExitCMPRecursionMultiplier(int footstepIndex)
   {
      return exitCMPRecursionMultiplier.getExitMultiplier(footstepIndex);
   }

   public double getEntryCMPRecursionMultiplier(int footstepIndex)
   {
      return entryCMPRecursionMultiplier.getEntryMultiplier(footstepIndex);
   }

   /*
   public void reset()
   {
      cmpRecursionMultipliers.reset();
      stanceCMPProjectionMultipliers.reset();
      finalICPRecursionMultiplier.reset();
      remainingStanceCMPProjectionMultipliers.reset();
      currentStateProjectionMultiplier.reset();
      initialICPProjectionMultiplier.reset();
   }

   public void computeRecursionMultipliers(int numberOfStepsToConsider, boolean isInTransfer, boolean useTwoCMPs, double omega0)
   {
      reset();

      if (numberOfStepsToConsider > maxNumberOfFootstepsToConsider)
         throw new RuntimeException("Requesting too many steps.");

      finalICPRecursionMultiplier.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0);
      stanceCMPProjectionMultipliers.compute(doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0, numberOfStepsToConsider);
      cmpRecursionMultipliers.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0);
   }

   public void computeRemainingProjectionMultipliers(double timeRemaining, boolean useTwoCMPs, boolean isInTransfer, double omega0, boolean useInitialICP)
   {
      if (useTwoCMPs)
      {
         updateSegmentedSingleSupportTrajectory(isInTransfer);
      }

      Math.max(timeRemaining, 0.0);

      currentStateProjectionMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
      initialICPProjectionMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
      remainingStanceCMPProjectionMultipliers.compute(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0, useInitialICP);
   }

   private void updateSegmentedSingleSupportTrajectory(boolean isInTransfer)
   {
      /
      if (!isInTransfer)
      {
         double doubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();
         double steppingDuration = singleSupportDurations.get(0).getDoubleValue() + doubleSupportDuration;

         double totalTimeSpentOnExitCMP = steppingDuration * exitCMPDurationInPercentOfStepTime.getDoubleValue();
         double totalTimeSpentOnEntryCMP = steppingDuration * (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue());

         double doubleSupportTimeSpentBeforeEntryCornerPoint = doubleSupportDuration * doubleSupportSplitFraction.getDoubleValue();
         double doubleSupportTimeSpentAfterEntryCornerPoint = doubleSupportDuration * (1.0 - doubleSupportSplitFraction.getDoubleValue());

         double timeRemainingOnEntryCMP = totalTimeSpentOnEntryCMP - doubleSupportTimeSpentBeforeEntryCornerPoint;
         double timeToSpendOnFinalCMPBeforeDoubleSupport = totalTimeSpentOnExitCMP - doubleSupportTimeSpentAfterEntryCornerPoint;

         timeSpentOnInitialCMP.set(timeRemainingOnEntryCMP);
         timeSpentOnFinalCMP.set(timeToSpendOnFinalCMPBeforeDoubleSupport);
         totalTrajectoryTime.set(timeRemainingOnEntryCMP + timeToSpendOnFinalCMPBeforeDoubleSupport);

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
      }
      else
      {
         timeSpentOnInitialCMP.set(Double.NaN);
         timeSpentOnFinalCMP.set(Double.NaN);
         totalTrajectoryTime.set(Double.NaN);
         startOfSplineTime.set(Double.NaN);
         endOfSplineTime.set(Double.NaN);
      }
   }

   public double getCMPRecursionExitMultiplier(int footstepIndex)
   {
      //return cmpRecursionMultipliers.getExitMultiplier(footstepIndex);
   }

   public double getCMPRecursionEntryMultiplier(int footstepIndex)
   {
      //return cmpRecursionMultipliers.getEntryMultiplier(footstepIndex);
   }

   public double getFinalICPRecursionMultiplier()
   {
      //return finalICPRecursionMultiplier.getDoubleValue();
   }

   public double getStanceExitCMPProjectionMultiplier()
   {
      return stanceCMPProjectionMultipliers.getExitMultiplier();
   }

   public double getStanceEntryCMPProjectionMultiplier()
   {
      return stanceCMPProjectionMultipliers.getEntryMultiplier();
   }

   public double getRemainingStanceExitCMPProjectionMultiplier()
   {
      return remainingStanceCMPProjectionMultipliers.getRemainingExitMultiplier();
   }

   public double getRemainingStanceEntryCMPProjectionMultiplier()
   {
      return remainingStanceCMPProjectionMultipliers.getRemainingEntryMultiplier();
   }

   public double getRemainingPreviousStanceExitCMPProjectionMultiplier()
   {
      return remainingStanceCMPProjectionMultipliers.getRemainingPreviousExitMultiplier();
   }

   public double getCurrentStateProjectionMultiplier()
   {
      return currentStateProjectionMultiplier.getPositionMultiplier();
   }

   public double getInitialICPProjectionMultiplier()
   {
      return initialICPProjectionMultiplier.getPositionMultiplier();
   }

   private final FramePoint2d tmpPoint = new FramePoint2d();
   private final FramePoint2d tmpEntry = new FramePoint2d();
   private final FramePoint2d tmpExit = new FramePoint2d();

   public void computeICPPoints(FramePoint2d finalICP, ArrayList<YoFramePoint2d> footstepLocations, ArrayList<FrameVector2d> entryOffsets,
         ArrayList<FrameVector2d> exitOffsets, FramePoint2d previousExitCMP, FramePoint2d entryCMP, FramePoint2d exitCMP, FramePoint2d initialICP, int numberOfFootstepsToConsider,
         FramePoint2d predictedEndOfStateICP, FramePoint2d referenceICPToPack, FrameVector2d referenceICPVelocityToPack)
   {
      predictedEndOfStateICP.set(finalICP);
      predictedEndOfStateICP.scale(getFinalICPRecursionMultiplier());

      tmpPoint.set(entryCMP);
      tmpPoint.scale(getStanceEntryCMPProjectionMultiplier());

      predictedEndOfStateICP.add(tmpPoint);

      tmpPoint.set(exitCMP);
      tmpPoint.scale(getStanceExitCMPProjectionMultiplier());

      predictedEndOfStateICP.add(tmpPoint);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         tmpEntry.set(footstepLocations.get(i).getFrameTuple2d());
         tmpExit.set(footstepLocations.get(i).getFrameTuple2d());

         tmpEntry.add(entryOffsets.get(i));
         tmpExit.add(exitOffsets.get(i));

         tmpEntry.scale(getCMPRecursionEntryMultiplier(i));
         tmpExit.scale(getCMPRecursionExitMultiplier(i));

         predictedEndOfStateICP.add(tmpEntry);
         predictedEndOfStateICP.add(tmpExit);
      }

      referenceICPToPack.set(predictedEndOfStateICP);
      referenceICPToPack.scale(currentStateProjectionMultiplier.getPositionMultiplier());

      if (!entryCMP.containsNaN())
      {
         tmpPoint.set(entryCMP);
         tmpPoint.scale(getRemainingStanceEntryCMPProjectionMultiplier());
         referenceICPToPack.add(tmpPoint);
      }

      if (!exitCMP.containsNaN())
      {
         tmpPoint.set(exitCMP);
         tmpPoint.scale(getRemainingStanceExitCMPProjectionMultiplier());
         referenceICPToPack.add(tmpPoint);
      }

      if (!previousExitCMP.containsNaN())
      {
         tmpPoint.set(previousExitCMP);
         tmpPoint.scale(getRemainingPreviousStanceExitCMPProjectionMultiplier());
         referenceICPToPack.add(tmpPoint);
      }

      if (!initialICP.containsNaN())
      {
         tmpPoint.set(initialICP);
         tmpPoint.scale(getInitialICPProjectionMultiplier());
         referenceICPToPack.add(tmpPoint);
      }

      referenceICPVelocityToPack.set(predictedEndOfStateICP);
      referenceICPVelocityToPack.scale(currentStateProjectionMultiplier.getVelocityMultiplier());

      if (!entryCMP.containsNaN())
      {
         tmpPoint.set(entryCMP);
         tmpPoint.scale(remainingStanceCMPProjectionMultipliers.getRemainingEntryVelocityMultiplier());
         referenceICPVelocityToPack.add(tmpPoint);
      }

      if (!exitCMP.containsNaN())
      {
         tmpPoint.set(exitCMP);
         tmpPoint.scale(remainingStanceCMPProjectionMultipliers.getRemainingExitVelocityMultiplier());
         referenceICPVelocityToPack.add(tmpPoint);
      }

      if (!previousExitCMP.containsNaN())
      {
         tmpPoint.set(previousExitCMP);
         tmpPoint.scale(remainingStanceCMPProjectionMultipliers.getRemainingPreviousExitVelocityMultiplier());
         referenceICPVelocityToPack.add(tmpPoint);
      }

      if (!initialICP.containsNaN())
      {
         tmpPoint.set(initialICP);
         tmpPoint.scale(initialICPProjectionMultiplier.getVelocityMultiplier());
         referenceICPVelocityToPack.add(tmpPoint);
      }
   }
   */
}
