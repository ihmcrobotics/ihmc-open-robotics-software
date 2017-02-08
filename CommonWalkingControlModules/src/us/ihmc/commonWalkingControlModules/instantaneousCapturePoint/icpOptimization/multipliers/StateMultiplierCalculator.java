package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current.*;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion.*;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;

import java.util.ArrayList;

public class StateMultiplierCalculator
{
   private static final boolean PROJECT_FORWARD = true;
   private static final String namePrefix = "controller";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

   private final DoubleYoVariable defaultDoubleSupportSplitFraction;
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   private final DoubleYoVariable maximumSplineDuration;
   private final DoubleYoVariable minimumSplineDuration;
   private final DoubleYoVariable minimumTimeToSpendOnExitCMP;
   private final DoubleYoVariable totalTrajectoryTime;
   private final DoubleYoVariable timeSpentOnInitialCMP;
   private final DoubleYoVariable timeSpentOnFinalCMP;
   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;
   private final IntegerYoVariable currentSwingSegment;

   private final FinalICPRecursionMultiplier finalICPRecursionMultiplier;
   private final StanceExitCMPRecursionMultiplier stanceExitCMPRecursionMultiplier;
   private final StanceEntryCMPRecursionMultiplier stanceEntryCMPRecursionMultiplier;
   private final ExitCMPRecursionMultiplier exitCMPRecursionMultiplier;
   private final EntryCMPRecursionMultiplier entryCMPRecursionMultiplier;

   private final ExitCMPCurrentMultiplier exitCMPCurrentMultiplier;
   private final EntryCMPCurrentMultiplier entryCMPCurrentMultiplier;
   private final InitialICPCurrentMultiplier initialICPCurrentMultiplier;
   private final InitialICPVelocityCurrentMultiplier initialICPVelocityCurrentMultiplier;
   private final StateEndCurrentMultiplier stateEndCurrentMultiplier;

   private final int maxNumberOfFootstepsToConsider;

   public StateMultiplierCalculator(CapturePointPlannerParameters icpPlannerParameters, DoubleYoVariable exitCMPDurationInPercentOfStepTime,
         DoubleYoVariable defaultDoubleSupportSplitFraction, DoubleYoVariable upcomingDoubleSupportSplitFraction, int maxNumberOfFootstepsToConsider, YoVariableRegistry parentRegistry)
   {
      this.maxNumberOfFootstepsToConsider = maxNumberOfFootstepsToConsider;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;
      this.defaultDoubleSupportSplitFraction = defaultDoubleSupportSplitFraction;

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
      currentSwingSegment = new IntegerYoVariable(namePrefix + "CurrentSegment", registry);


      finalICPRecursionMultiplier = new FinalICPRecursionMultiplier(namePrefix, exitCMPDurationInPercentOfStepTime, registry);
      stanceExitCMPRecursionMultiplier = new StanceExitCMPRecursionMultiplier(namePrefix, exitCMPDurationInPercentOfStepTime, registry);
      stanceEntryCMPRecursionMultiplier = new StanceEntryCMPRecursionMultiplier(namePrefix, exitCMPDurationInPercentOfStepTime, registry);
      exitCMPRecursionMultiplier = new ExitCMPRecursionMultiplier(namePrefix, maxNumberOfFootstepsToConsider, exitCMPDurationInPercentOfStepTime, registry);
      entryCMPRecursionMultiplier = new EntryCMPRecursionMultiplier(namePrefix, maxNumberOfFootstepsToConsider, exitCMPDurationInPercentOfStepTime, registry);

      exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(upcomingDoubleSupportSplitFraction, exitCMPDurationInPercentOfStepTime, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime, registry);
      entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(upcomingDoubleSupportSplitFraction, defaultDoubleSupportSplitFraction,
            exitCMPDurationInPercentOfStepTime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime, PROJECT_FORWARD, registry);
      initialICPCurrentMultiplier = new InitialICPCurrentMultiplier(upcomingDoubleSupportSplitFraction, defaultDoubleSupportSplitFraction,
            exitCMPDurationInPercentOfStepTime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime, PROJECT_FORWARD, registry);
      initialICPVelocityCurrentMultiplier = new InitialICPVelocityCurrentMultiplier(registry);
      stateEndCurrentMultiplier = new StateEndCurrentMultiplier(upcomingDoubleSupportSplitFraction, defaultDoubleSupportSplitFraction,
            exitCMPDurationInPercentOfStepTime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime, PROJECT_FORWARD, registry);

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


   public void resetCurrentMultipliers()
   {
      exitCMPCurrentMultiplier.reset();
      entryCMPCurrentMultiplier.reset();
      initialICPCurrentMultiplier.reset();
      initialICPVelocityCurrentMultiplier.reset();
      stateEndCurrentMultiplier.reset();
   }

   public void computeCurrentMultipliers(double timeInState, boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      resetCurrentMultipliers();

      if (useTwoCMPs)
      {
         updateSegmentedSingleSupportTrajectory(timeInState, isInTransfer);
      }

      exitCMPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeInState, useTwoCMPs, isInTransfer, omega0);
      entryCMPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeInState, useTwoCMPs, isInTransfer, omega0);
      initialICPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeInState, useTwoCMPs, isInTransfer, omega0);
      initialICPVelocityCurrentMultiplier.compute(doubleSupportDurations, timeInState, isInTransfer);
      stateEndCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeInState, useTwoCMPs, isInTransfer, omega0);
   }

   private void updateSegmentedSingleSupportTrajectory(double timeInState, boolean isInTransfer)
   {
      if (!isInTransfer)
      {
         double doubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();
         double steppingDuration = singleSupportDurations.get(0).getDoubleValue() + doubleSupportDuration;

         double totalTimeSpentOnExitCMP = steppingDuration * exitCMPDurationInPercentOfStepTime.getDoubleValue();
         double totalTimeSpentOnEntryCMP = steppingDuration * (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue());

         double doubleSupportTimeSpentBeforeEntryCornerPoint = doubleSupportDuration * defaultDoubleSupportSplitFraction.getDoubleValue();
         double doubleSupportTimeSpentAfterEntryCornerPoint = doubleSupportDuration * (1.0 - defaultDoubleSupportSplitFraction.getDoubleValue());

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

         if (timeInState <= startOfSplineTime)
            currentSwingSegment.set(1);
         else if (timeInState >= endOfSplineTime)
            currentSwingSegment.set(3);
         else
            currentSwingSegment.set(2);
      }
      else
      {
         currentSwingSegment.set(0);

         timeSpentOnInitialCMP.set(Double.NaN);
         timeSpentOnFinalCMP.set(Double.NaN);
         totalTrajectoryTime.set(Double.NaN);
         startOfSplineTime.set(Double.NaN);
         endOfSplineTime.set(Double.NaN);
      }

   }

   public double getExitCMPCurrentMultiplier()
   {
      return exitCMPCurrentMultiplier.getPositionMultiplier();
   }

   public double getEntryCMPCurrentMultiplier()
   {
      return entryCMPCurrentMultiplier.getPositionMultiplier();
   }

   public double getInitialICPCurrentMultiplier()
   {
      return initialICPCurrentMultiplier.getPositionMultiplier();
   }

   public double getInitialICPVelocityCurrentMultiplier()
   {
      return initialICPVelocityCurrentMultiplier.getPositionMultiplier();
   }

   public double getStateEndCurrentMultiplier()
   {
      return stateEndCurrentMultiplier.getPositionMultiplier();
   }

   private final FramePoint2d tmpPoint = new FramePoint2d();
   private final FramePoint2d tmpEntry = new FramePoint2d();
   private final FramePoint2d tmpExit = new FramePoint2d();

   public void reconstructICPCornerPoint(FramePoint2d predictedICPCornerPoint, FramePoint2d finalICP, ArrayList<FramePoint2d> footstepLocations,
         ArrayList<FrameVector2d> entryOffsets, ArrayList<FrameVector2d> exitOffsets, FramePoint2d entryCMP, FramePoint2d exitCMP, int numberOfFootstepsToConsider)
   {
      predictedICPCornerPoint.set(finalICP);
      predictedICPCornerPoint.scale(getFinalICPRecursionMultiplier());

      tmpPoint.set(entryCMP);
      tmpPoint.scale(getStanceEntryCMPRecursionMultiplier());
      predictedICPCornerPoint.add(tmpPoint);

      tmpPoint.set(exitCMP);
      tmpPoint.scale(getStanceExitCMPRecursionMultiplier());
      predictedICPCornerPoint.add(tmpPoint);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         tmpEntry.set(footstepLocations.get(i));
         tmpExit.set(footstepLocations.get(i));

         tmpEntry.add(entryOffsets.get(i));
         tmpExit.add(exitOffsets.get(i));

         tmpEntry.scale(getEntryCMPRecursionMultiplier(i));
         tmpExit.scale(getExitCMPRecursionMultiplier(i));

         predictedICPCornerPoint.add(tmpEntry);
         predictedICPCornerPoint.add(tmpExit);
      }
   }

   public void yoReconstructICPCornerPoint(FramePoint2d predictedICPCornerPoint, FramePoint2d finalICP, ArrayList<YoFramePoint2d> footstepLocations,
         ArrayList<FrameVector2d> entryOffsets, ArrayList<FrameVector2d> exitOffsets, FramePoint2d entryCMP, FramePoint2d exitCMP, int numberOfFootstepsToConsider)
   {
      predictedICPCornerPoint.set(finalICP);
      predictedICPCornerPoint.scale(getFinalICPRecursionMultiplier());

      tmpPoint.set(entryCMP);
      tmpPoint.scale(getStanceEntryCMPRecursionMultiplier());
      predictedICPCornerPoint.add(tmpPoint);

      tmpPoint.set(exitCMP);
      tmpPoint.scale(getStanceExitCMPRecursionMultiplier());
      predictedICPCornerPoint.add(tmpPoint);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         tmpEntry.set(footstepLocations.get(i).getFrameTuple2d());
         tmpExit.set(footstepLocations.get(i).getFrameTuple2d());

         tmpEntry.add(entryOffsets.get(i));
         tmpExit.add(exitOffsets.get(i));

         tmpEntry.scale(getEntryCMPRecursionMultiplier(i));
         tmpExit.scale(getExitCMPRecursionMultiplier(i));

         predictedICPCornerPoint.add(tmpEntry);
         predictedICPCornerPoint.add(tmpExit);
      }
   }

   public void reconstructReferenceICP(FramePoint2d referenceICPToPack, FrameVector2d referenceICPVelocityToPack, FramePoint2d predictedICPCornerPoint,
         FramePoint2d entryCMP, FramePoint2d exitCMP, FramePoint2d initialICP, FrameVector2d initialICPVelocity)
   {
      referenceICPToPack.set(predictedICPCornerPoint);
      referenceICPToPack.scale(stateEndCurrentMultiplier.getPositionMultiplier());

      referenceICPVelocityToPack.set(predictedICPCornerPoint);
      referenceICPVelocityToPack.scale(stateEndCurrentMultiplier.getVelocityMultiplier());

      if (!entryCMP.containsNaN())
      {
         tmpPoint.setToZero();
         tmpPoint.set(entryCMP);
         tmpPoint.scale(entryCMPCurrentMultiplier.getPositionMultiplier());
         referenceICPToPack.add(tmpPoint);

         tmpPoint.setToZero();
         tmpPoint.set(entryCMP);
         tmpPoint.scale(entryCMPCurrentMultiplier.getVelocityMultiplier());
         referenceICPVelocityToPack.add(tmpPoint);
      }

      if (!exitCMP.containsNaN())
      {
         tmpPoint.setToZero();
         tmpPoint.set(exitCMP);
         tmpPoint.scale(exitCMPCurrentMultiplier.getPositionMultiplier());
         referenceICPToPack.add(tmpPoint);

         tmpPoint.setToZero();
         tmpPoint.set(exitCMP);
         tmpPoint.scale(exitCMPCurrentMultiplier.getVelocityMultiplier());
         referenceICPVelocityToPack.add(tmpPoint);
      }

      if (!initialICP.containsNaN())
      {
         tmpPoint.setToZero();
         tmpPoint.set(initialICP);
         tmpPoint.scale(initialICPCurrentMultiplier.getPositionMultiplier());
         referenceICPToPack.add(tmpPoint);

         tmpPoint.setToZero();
         tmpPoint.set(initialICP);
         tmpPoint.scale(initialICPCurrentMultiplier.getVelocityMultiplier());
         referenceICPVelocityToPack.add(tmpPoint);
      }

      if (!initialICPVelocity.containsNaN())
      {
         tmpPoint.setToZero();
         tmpPoint.set(initialICPVelocity);
         tmpPoint.scale(initialICPVelocityCurrentMultiplier.getPositionMultiplier());
         referenceICPToPack.add(tmpPoint);

         tmpPoint.setToZero();
         tmpPoint.set(initialICPVelocity);
         tmpPoint.scale(initialICPVelocityCurrentMultiplier.getVelocityMultiplier());
         referenceICPVelocityToPack.add(tmpPoint);
      }
   }
}
