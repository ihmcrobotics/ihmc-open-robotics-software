package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current.*;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion.*;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;

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

   private final NewRecursionMultipliers newRecursionMultipliers;

   private final NewExitCMPCurrentMultiplier newExitCMPCurrentMultiplier;
   private final NewEntryCMPCurrentMultiplier newEntryCMPCurrentMultiplier;
   private final NewInitialICPCurrentMultiplier newInitialICPCurrentMultiplier;
   private final NewInitialICPVelocityCurrentMultiplier newInitialICPVelocityCurrentMultiplier;
   private final NewStateEndCurrentMultiplier newStateEndCurrentMultiplier;

   private final CubicMatrix cubicMatrix;
   private final CubicDerivativeMatrix cubicDerivativeMatrix;

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


      cubicMatrix = new CubicMatrix();
      cubicDerivativeMatrix = new CubicDerivativeMatrix();

      boolean clipTime = true;

      newExitCMPCurrentMultiplier = new NewExitCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, cubicMatrix,
            cubicDerivativeMatrix, yoNamePrefix, clipTime, registry);
      newEntryCMPCurrentMultiplier = new NewEntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime, cubicMatrix, cubicDerivativeMatrix, yoNamePrefix, clipTime, registry);
      newInitialICPCurrentMultiplier = new NewInitialICPCurrentMultiplier(startOfSplineTime, endOfSplineTime, cubicMatrix, cubicDerivativeMatrix, yoNamePrefix,
            registry);
      newInitialICPVelocityCurrentMultiplier = new NewInitialICPVelocityCurrentMultiplier(cubicMatrix, cubicDerivativeMatrix, yoNamePrefix, registry);
      newStateEndCurrentMultiplier = new NewStateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime,
            cubicMatrix, cubicDerivativeMatrix, yoNamePrefix, clipTime, registry);

      newRecursionMultipliers = new NewRecursionMultipliers(yoNamePrefix, maxNumberOfFootstepsToConsider, swingSplitFractions, transferSplitFractions,
            registry);

      parentRegistry.addChild(registry);
   }

   public void resetRecursionMultipliers()
   {
      newRecursionMultipliers.reset();
   }

   public void computeRecursionMultipliers(int numberOfStepsToConsider, int numberOfStepsRegistered, boolean isInTransfer, boolean useTwoCMPs, double omega0)
   {
      resetRecursionMultipliers();

      if (numberOfStepsToConsider > maxNumberOfFootstepsToConsider)
         throw new RuntimeException("Requesting too many steps.");

      newRecursionMultipliers.compute(numberOfStepsToConsider, numberOfStepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega0);
   }

   public double getFinalICPRecursionMultiplier()
   {
      return newRecursionMultipliers.getFinalICPMultiplier();
   }

   public double getStanceExitCMPRecursionMultiplier()
   {
      return newRecursionMultipliers.getStanceExitMultiplier();
   }

   public double getStanceEntryCMPRecursionMultiplier()
   {
      return newRecursionMultipliers.getStanceEntryMultiplier();
   }

   public double getExitCMPRecursionMultiplier(int footstepIndex)
   {
      return newRecursionMultipliers.getExitMultiplier(footstepIndex);
   }

   public double getEntryCMPRecursionMultiplier(int footstepIndex)
   {
      return newRecursionMultipliers.getEntryMultiplier(footstepIndex);
   }


   public void resetCurrentMultipliers()
   {
      newExitCMPCurrentMultiplier.reset();
      newEntryCMPCurrentMultiplier.reset();
      newInitialICPCurrentMultiplier.reset();
      newInitialICPVelocityCurrentMultiplier.reset();
      newStateEndCurrentMultiplier.reset();
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

      if (useTwoCMPs && !isInTransfer)
         updateSegmentedSingleSupportTrajectory(timeInState);

      double timeInSpline;
      if (isInTransfer)
         timeInSpline = timeInState;
      else
         timeInSpline = timeInState - startOfSplineTime.getDoubleValue();

      cubicMatrix.update(timeInSpline);
      cubicDerivativeMatrix.update(timeInSpline);

      newExitCMPCurrentMultiplier.compute(numberOfFootstepsToConsider, singleSupportDurations, doubleSupportDurations, timeInState, useTwoCMPs, isInTransfer, omega0);
      newEntryCMPCurrentMultiplier.compute(numberOfFootstepsToConsider, singleSupportDurations, doubleSupportDurations, timeInState, useTwoCMPs, isInTransfer, omega0);
      newInitialICPCurrentMultiplier.compute(doubleSupportDurations, timeInState, useTwoCMPs, isInTransfer, omega0);
      newInitialICPVelocityCurrentMultiplier.compute(doubleSupportDurations, timeInState, isInTransfer);
      newStateEndCurrentMultiplier.compute(numberOfFootstepsToConsider, singleSupportDurations, doubleSupportDurations, timeInState, useTwoCMPs, isInTransfer, omega0);
   }

   private void updateSegmentedSingleSupportTrajectory(double timeInState)
   {
      if (timeInState <= startOfSplineTime.getDoubleValue())
         currentSwingSegment.set(1);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         currentSwingSegment.set(3);
      else
         currentSwingSegment.set(2);
   }

   public double getExitCMPCurrentMultiplier()
   {
      return newExitCMPCurrentMultiplier.getPositionMultiplier();
   }

   public double getExitCMPCurrentVelocityMultiplier()
   {
      return newExitCMPCurrentMultiplier.getVelocityMultiplier();
   }

   public double getEntryCMPCurrentMultiplier()
   {
      return newEntryCMPCurrentMultiplier.getPositionMultiplier();
   }

   public double getEntryCMPCurrentVelocityMultiplier()
   {
      return newEntryCMPCurrentMultiplier.getVelocityMultiplier();
   }

   public double getInitialICPCurrentMultiplier()
   {
      return newInitialICPCurrentMultiplier.getPositionMultiplier();
   }

   public double getInitialICPCurrentVelocityMultiplier()
   {
      return newInitialICPCurrentMultiplier.getVelocityMultiplier();
   }

   public double getInitialICPVelocityCurrentMultiplier()
   {
      return newInitialICPVelocityCurrentMultiplier.getPositionMultiplier();
   }

   public double getInitialICPVelocityCurrentVelocityMultiplier()
   {
      return newInitialICPVelocityCurrentMultiplier.getVelocityMultiplier();
   }

   public double getStateEndCurrentMultiplier()
   {
      return newStateEndCurrentMultiplier.getPositionMultiplier();
   }

   public double getStateEndCurrentVelocityMultiplier()
   {
      return newStateEndCurrentMultiplier.getVelocityMultiplier();
   }

   private final FramePoint2d tmpPoint = new FramePoint2d();
   private final FramePoint2d tmpEntry = new FramePoint2d();
   private final FramePoint2d tmpExit = new FramePoint2d();

   public void reconstructICPCornerPoint(FramePoint2d predictedICPCornerPointToPack, FramePoint2d finalICP, ArrayList<FramePoint2d> footstepLocations,
         ArrayList<FrameVector2d> entryOffsets, ArrayList<FrameVector2d> exitOffsets, FramePoint2d entryCMP, FramePoint2d exitCMP, int numberOfFootstepsToConsider)
   {
      predictedICPCornerPointToPack.set(finalICP);
      predictedICPCornerPointToPack.scale(getFinalICPRecursionMultiplier());

      tmpPoint.set(entryCMP);
      tmpPoint.scale(getStanceEntryCMPRecursionMultiplier());
      predictedICPCornerPointToPack.add(tmpPoint);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         tmpEntry.set(footstepLocations.get(i));
         tmpEntry.add(entryOffsets.get(i));
         tmpEntry.scale(getEntryCMPRecursionMultiplier(i));
         predictedICPCornerPointToPack.add(tmpEntry);
      }

      if (!exitCMP.containsNaN())
      {
         tmpPoint.set(exitCMP);
         tmpPoint.scale(getStanceExitCMPRecursionMultiplier());
         predictedICPCornerPointToPack.add(tmpPoint);

         for (int i = 0; i < numberOfFootstepsToConsider; i++)
         {
            tmpExit.set(footstepLocations.get(i));
            tmpExit.add(exitOffsets.get(i));
            tmpExit.scale(getExitCMPRecursionMultiplier(i));
            predictedICPCornerPointToPack.add(tmpExit);
         }
      }
   }

   public void yoReconstructICPCornerPoint(FramePoint2d predictedICPCornerPointToPack, FramePoint2d finalICP, ArrayList<YoFramePoint2d> footstepLocations,
         ArrayList<FrameVector2d> entryOffsets, ArrayList<FrameVector2d> exitOffsets, FramePoint2d entryCMP, FramePoint2d exitCMP, int numberOfFootstepsToConsider)
   {
      predictedICPCornerPointToPack.set(finalICP);
      predictedICPCornerPointToPack.scale(getFinalICPRecursionMultiplier());

      tmpPoint.set(entryCMP);
      tmpPoint.scale(getStanceEntryCMPRecursionMultiplier());
      predictedICPCornerPointToPack.add(tmpPoint);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         tmpEntry.set(footstepLocations.get(i).getFrameTuple2d());
         tmpEntry.add(entryOffsets.get(i));
         tmpEntry.scale(getEntryCMPRecursionMultiplier(i));
         predictedICPCornerPointToPack.add(tmpEntry);
      }

      if (!exitCMP.containsNaN())
      {
         tmpPoint.set(exitCMP);
         tmpPoint.scale(getStanceExitCMPRecursionMultiplier());
         predictedICPCornerPointToPack.add(tmpPoint);

         for (int i = 0; i < numberOfFootstepsToConsider; i++)
         {
            tmpExit.set(footstepLocations.get(i).getFrameTuple2d());
            tmpExit.add(exitOffsets.get(i));
            tmpExit.scale(getExitCMPRecursionMultiplier(i));
            predictedICPCornerPointToPack.add(tmpExit);
         }
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
