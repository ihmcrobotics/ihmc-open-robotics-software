package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import java.util.List;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class StanceEntryCMPRecursionMultiplier
{
   private static final String name = "StanceCMPEntryRecursionMultiplier";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable entryMultiplier;
   private final List<DoubleYoVariable> swingSplitFractions;
   private final List<DoubleYoVariable> transferSplitFractions;

   public StanceEntryCMPRecursionMultiplier(String namePrefix, List<DoubleYoVariable> swingSplitFractions, List<DoubleYoVariable> transferSplitFractions,
         YoVariableRegistry parentRegistry)
   {
      this.swingSplitFractions = swingSplitFractions;
      this.transferSplitFractions = transferSplitFractions;

      entryMultiplier = new DoubleYoVariable(namePrefix + name, registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      entryMultiplier.setToNaN();
   }

   public void compute(int numberOfFootstepsToConsider, List<DoubleYoVariable> doubleSupportDurations, List<DoubleYoVariable> singleSupportDurations,
         boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      if (numberOfFootstepsToConsider == 0)
      {
         this.entryMultiplier.set(0.0);
         return;
      }

      if (useTwoCMPs)
         computeWithTwoCMPs(doubleSupportDurations, singleSupportDurations, isInTransfer, omega0);
      else
         computeWithOneCMP(doubleSupportDurations, singleSupportDurations, omega0);
   }

   private void computeWithOneCMP(List<DoubleYoVariable> doubleSupportDurations, List<DoubleYoVariable> singleSupportDurations, double omega0)
   {
      double currentTransferOnCMP = (1.0 - transferSplitFractions.get(0).getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();
      double currentSwingOnCMP = singleSupportDurations.get(0).getDoubleValue();
      double nextTransferOnCMP = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

      double timeOnCurrentCMP = currentTransferOnCMP + currentSwingOnCMP + nextTransferOnCMP;

      double entryMultiplier = 1.0 - Math.exp(-omega0 * timeOnCurrentCMP);
      this.entryMultiplier.set(entryMultiplier);
   }

   private void computeWithTwoCMPs(List<DoubleYoVariable> doubleSupportDurations, List<DoubleYoVariable> singleSupportDurations, boolean isInTransfer, double omega0)
   {
      double currentTransferOnEntry = (1.0 - transferSplitFractions.get(0).getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();
      double currentSwingOnEntry = swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();

      double timeSpentOnEntryCMP = currentTransferOnEntry + currentSwingOnEntry;

      if (isInTransfer)
      {
         double entryMultiplier = 1.0 - Math.exp(-omega0 * timeSpentOnEntryCMP);
         this.entryMultiplier.set(entryMultiplier);
      }
      else
      {
         this.entryMultiplier.set(0.0);
      }
   }

   public double getEntryMultiplier()
   {
      return entryMultiplier.getDoubleValue();
   }
}
