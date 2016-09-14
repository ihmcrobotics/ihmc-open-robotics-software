package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class StanceCMPProjectionMultipliers
{
   private static final String entryName = "StanceCMPEntryProjectionMultiplier";
   private static final String exitName = "StanceCMPExitProjectionMultiplier";
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable exitMultiplier;
   private final DoubleYoVariable entryMultiplier;

   private final DoubleYoVariable omega;
   private final DoubleYoVariable doubleSupportSplitFraction;
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   public StanceCMPProjectionMultipliers(String namePrefix, DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitFraction,
         DoubleYoVariable exitCMPDurationInPercentOfStepTime, YoVariableRegistry parentRegistry)
   {
      this.omega = omega;
      this.doubleSupportSplitFraction = doubleSupportSplitFraction;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;

      exitMultiplier = new DoubleYoVariable(namePrefix + exitName, registry);
      entryMultiplier = new DoubleYoVariable(namePrefix + entryName, registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      exitMultiplier.set(0.0);
      entryMultiplier.set(0.0);
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, boolean useTwoCMPs, boolean isInTransfer)
   {
      double upcomingDoubleSupportDuration = doubleSupportDurations.get(1).getDoubleValue();
      double currentDoubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();
      double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();

      compute(upcomingDoubleSupportDuration, currentDoubleSupportDuration, singleSupportDuration, useTwoCMPs, isInTransfer);
   }

   public void compute(double upcomingDoubleSupportDuration, double currentDoubleSupportDuration, double singleSupportDuration, boolean useTwoCMPs, boolean isInTransfer)
   {
      double firstStepTime = currentDoubleSupportDuration + singleSupportDuration;
      double timeSpentOnInitialDoubleSupportUpcoming = doubleSupportSplitFraction.getDoubleValue() * upcomingDoubleSupportDuration;
      double timeSpentOnEndDoubleSupportCurrent = (1.0 - doubleSupportSplitFraction.getDoubleValue()) * currentDoubleSupportDuration;

      if (useTwoCMPs)
      {

         if (isInTransfer)
         {
            double totalTimeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * firstStepTime;
            double totalTimeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * firstStepTime;

            double projectionTime = totalTimeSpentOnEntryCMP - timeSpentOnEndDoubleSupportCurrent;
            double multiplier = Math.exp(-omega.getDoubleValue() * projectionTime);

            exitMultiplier.set(multiplier * (1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnExitCMP)));
            entryMultiplier.set(1.0 - multiplier);
         }
         else
         {
            exitMultiplier.set(1.0 - Math.exp(-omega.getDoubleValue() * timeSpentOnInitialDoubleSupportUpcoming));
            entryMultiplier.set(0.0);
         }
      }
      else
      {
         double timeToFinish = timeSpentOnInitialDoubleSupportUpcoming;

         if (isInTransfer)
            timeToFinish += singleSupportDuration;

         exitMultiplier.set(1.0 - Math.exp(-omega.getDoubleValue() * timeToFinish));
         entryMultiplier.set(0.0);
      }

   }

   public double getExitMultiplier()
   {
      return exitMultiplier.getDoubleValue();
   }

   public double getEntryMultiplier()
   {
      return entryMultiplier.getDoubleValue();
   }
}
