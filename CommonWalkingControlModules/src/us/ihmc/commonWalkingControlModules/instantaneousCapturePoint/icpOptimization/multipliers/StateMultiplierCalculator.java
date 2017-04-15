package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current.EntryCMPCurrentMultiplier;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current.ExitCMPCurrentMultiplier;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current.InitialICPCurrentMultiplier;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current.InitialICPVelocityCurrentMultiplier;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current.StateEndCurrentMultiplier;
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
   private static final boolean PROJECT_FORWARD = true;
   private static final String namePrefix = "controller";

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

   private final CubicMatrix cubicMatrix;
   private final CubicDerivativeMatrix cubicDerivativeMatrix;

   private final int maxNumberOfFootstepsToConsider;

   public StateMultiplierCalculator(CapturePointPlannerParameters icpPlannerParameters, List<DoubleYoVariable> doubleSupportDurations,
         List<DoubleYoVariable> singleSupportDurations, List<DoubleYoVariable> transferSplitFractions,
         List<DoubleYoVariable> swingSplitFractions, int maxNumberOfFootstepsToConsider, YoVariableRegistry parentRegistry)
   {
      this.maxNumberOfFootstepsToConsider = maxNumberOfFootstepsToConsider;
      this.doubleSupportDurations = doubleSupportDurations;
      this.singleSupportDurations = singleSupportDurations;
      this.swingSplitFractions = swingSplitFractions;

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

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

      recursionMultipliers = new RecursionMultipliers(namePrefix, maxNumberOfFootstepsToConsider, swingSplitFractions, transferSplitFractions, registry);

      cubicMatrix = new CubicMatrix();
      cubicDerivativeMatrix = new CubicDerivativeMatrix();

      exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, startOfSplineTime, endOfSplineTime, cubicMatrix, cubicDerivativeMatrix,
            registry);
      entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime, cubicMatrix, cubicDerivativeMatrix, PROJECT_FORWARD, registry);
      initialICPCurrentMultiplier = new InitialICPCurrentMultiplier(swingSplitFractions, startOfSplineTime, endOfSplineTime, cubicMatrix, cubicDerivativeMatrix,
            PROJECT_FORWARD, registry);
      initialICPVelocityCurrentMultiplier = new InitialICPVelocityCurrentMultiplier(cubicMatrix, cubicDerivativeMatrix, registry);
      stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, cubicMatrix,
            cubicDerivativeMatrix, PROJECT_FORWARD, registry);

      parentRegistry.addChild(registry);
   }

   public void resetRecursionMultipliers()
   {
      recursionMultipliers.reset();
   }

   public void computeRecursionMultipliers(int numberOfStepsToConsider, int numberOfStepsRegistered, boolean isInTransfer, boolean useTwoCMPs, double omega0)
   {
      resetRecursionMultipliers();

      if (numberOfStepsToConsider > maxNumberOfFootstepsToConsider)
         throw new RuntimeException("Requesting too many steps.");

         recursionMultipliers.compute(numberOfStepsToConsider, numberOfStepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0);
   }

   public double getFinalICPRecursionMultiplier()
   {
      return recursionMultipliers.getFinalICPMultiplier();
   }

   public double getStanceExitCMPRecursionMultiplier()
   {
      return recursionMultipliers.getStanceExitMultiplier();
   }

   public double getStanceEntryCMPRecursionMultiplier()
   {
      return recursionMultipliers.getStanceEntryMultiplier();
   }

   public double getExitCMPRecursionMultiplier(int footstepIndex)
   {
      return recursionMultipliers.getExitMultiplier(footstepIndex);
   }

   public double getEntryCMPRecursionMultiplier(int footstepIndex)
   {
      return recursionMultipliers.getEntryMultiplier(footstepIndex);
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

   public void computeCurrentMultipliers(double timeInState, boolean useTwoCMPs, boolean isInTransfer, double omega0)
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

      exitCMPCurrentMultiplier.compute(singleSupportDurations, timeInState, useTwoCMPs, isInTransfer, omega0);
      entryCMPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeInState, useTwoCMPs, isInTransfer, omega0);
      initialICPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeInState, useTwoCMPs, isInTransfer, omega0);
      initialICPVelocityCurrentMultiplier.compute(doubleSupportDurations, timeInState, isInTransfer);
      stateEndCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeInState, useTwoCMPs, isInTransfer, omega0);
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
      referenceICPToPack.scale(stateEndCurrentMultiplier.getPositionMultiplier());

      referenceICPVelocityToPack.set(predictedICPCornerPoint);
      referenceICPVelocityToPack.scale(stateEndCurrentMultiplier.getVelocityMultiplier());

      tmpPoint.setToZero();
      tmpPoint.set(entryCMP);
      tmpPoint.scale(entryCMPCurrentMultiplier.getPositionMultiplier());
      referenceICPToPack.add(tmpPoint);

      tmpPoint.setToZero();
      tmpPoint.set(entryCMP);
      tmpPoint.scale(entryCMPCurrentMultiplier.getVelocityMultiplier());
      referenceICPVelocityToPack.add(tmpPoint);

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

      tmpPoint.setToZero();
      tmpPoint.set(initialICP);
      tmpPoint.scale(initialICPCurrentMultiplier.getPositionMultiplier());
      referenceICPToPack.add(tmpPoint);

      tmpPoint.setToZero();
      tmpPoint.set(initialICP);
      tmpPoint.scale(initialICPCurrentMultiplier.getVelocityMultiplier());
      referenceICPVelocityToPack.add(tmpPoint);

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
