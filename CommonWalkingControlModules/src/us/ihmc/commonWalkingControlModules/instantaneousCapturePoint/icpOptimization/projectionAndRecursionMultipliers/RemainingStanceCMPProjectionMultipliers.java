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

   public RemainingStanceCMPProjectionMultipliers(DoubleYoVariable defaultDoubleSupportSplitRatio, DoubleYoVariable upcomingDoubleSupportSplitRatio,
         DoubleYoVariable exitCMPDurationInPercentOfStepTime, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime,
         DoubleYoVariable totalTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      exitMultiplier = new ExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio, exitCMPDurationInPercentOfStepTime, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime);
      entryMultiplier = new EntryCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, exitCMPDurationInPercentOfStepTime, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime);
      previousExitMultiplier = new PreviousExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      exitMultiplier.reset();
      entryMultiplier.reset();
      previousExitMultiplier.reset();
   }

   public void compute(double timeRemaining, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean useTwoCMPs, boolean isInTransfer, double omega0, boolean useInitialICP)
   {
      timeRemaining = Math.max(timeRemaining, 0.0);
      exitMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
      entryMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
      previousExitMultiplier.compute(doubleSupportDurations, timeRemaining, isInTransfer, omega0, useInitialICP);
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
