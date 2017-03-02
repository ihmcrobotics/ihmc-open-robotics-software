package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class StanceExitCMPRecursionMultiplier
{
   private static final String name = "StanceCMPExitRecursionMultiplier";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable exitMultiplier;
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   public StanceExitCMPRecursionMultiplier(String namePrefix, DoubleYoVariable exitCMPDurationInPercentOfStepTime,
         YoVariableRegistry parentRegistry)
   {
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;

      exitMultiplier = new DoubleYoVariable(namePrefix + name, registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      exitMultiplier.set(0.0);
   }

   public void compute(int numberOfFootstepsToConsider, ArrayList<DoubleYoVariable> doubleSupportDurations,
         ArrayList<DoubleYoVariable> singleSupportDurations, boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      if (numberOfFootstepsToConsider == 0)
      {
         this.exitMultiplier.set(0.0);
         return;
      }

      if (useTwoCMPs)
         computeWithTwoCMPs(doubleSupportDurations, singleSupportDurations, isInTransfer, omega0);
      else
         computeWithOneCMP();
   }

   private void computeWithOneCMP()
   {
      exitMultiplier.set(0.0);
   }

   private void computeWithTwoCMPs(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, boolean isInTransfer, double omega0)
   {
      double firstStepTime = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
      double timeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * firstStepTime;
      double timeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * firstStepTime;

      if (isInTransfer)
      {
         double exitMultiplier = Math.exp(-omega0 * timeSpentOnEntryCMP) * (1.0 - Math.exp(-omega0 * timeSpentOnExitCMP));
         this.exitMultiplier.set(exitMultiplier);
      }
      else
      {
         double exitMultiplier = 1.0 - Math.exp(-omega0 * timeSpentOnExitCMP);
         this.exitMultiplier.set(exitMultiplier);
      }
   }

   public double getExitMultiplier()
   {
      return exitMultiplier.getDoubleValue();
   }
}
