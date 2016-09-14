package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class RemainingStanceCMPProjectionMultipliers
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ExitCMPProjectionMultiplier exitMultiplier;
   private final EntryCMPProjectionMultiplier entryMultiplier;
   private final PreviousExitCMPProjectionMultiplier previousExitMultiplier;

   public RemainingStanceCMPProjectionMultipliers(DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio,
         DoubleYoVariable exitCMPDurationInPercentOfStepTime, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime,
         DoubleYoVariable totalTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      exitMultiplier = new ExitCMPProjectionMultiplier(registry, omega, doubleSupportSplitRatio, exitCMPDurationInPercentOfStepTime, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime);
      entryMultiplier = new EntryCMPProjectionMultiplier(registry, omega, doubleSupportSplitRatio, exitCMPDurationInPercentOfStepTime, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime);
      previousExitMultiplier = new PreviousExitCMPProjectionMultiplier(registry, omega, doubleSupportSplitRatio);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      exitMultiplier.reset();
      entryMultiplier.reset();
      previousExitMultiplier.reset();
   }

   public void compute(double timeRemaining, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
                       boolean useTwoCMPs, boolean isInTransfer)
   {
      exitMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer);
      entryMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer);
      previousExitMultiplier.compute(doubleSupportDurations, timeRemaining, isInTransfer);
   }

   public double getRemainingExitMultiplier()
   {
      return exitMultiplier.getPositionMultiplier();
   }

   public double getRemainingEntryMultiplier()
   {
      return entryMultiplier.getPositionMultiplier();
   }

   public double getRemainingPreviousExitMultiplier()
   {
      return previousExitMultiplier.getPositionMultiplier();
   }

   public double getRemainingExitVelocityMultiplier()
   {
      return exitMultiplier.getVelocityMultiplier();
   }

   public double getRemainingEntryVelocityMultiplier()
   {
      return entryMultiplier.getVelocityMultiplier();
   }

   public double getRemainingPreviousExitVelocityMultiplier()
   {
      return previousExitMultiplier.getVelocityMultiplier();
   }
}
