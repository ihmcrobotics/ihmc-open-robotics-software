package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import java.util.List;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class StanceExitCMPRecursionMultiplier
{
   private static final String name = "StanceCMPExitRecursionMultiplier";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable exitMultiplier;
   private final List<DoubleYoVariable> swingSplitFractions;
   private final List<DoubleYoVariable> transferSplitFractions;

   public StanceExitCMPRecursionMultiplier(String namePrefix, List<DoubleYoVariable> swingSplitFractions, List<DoubleYoVariable> transferSplitFractions,
         YoVariableRegistry parentRegistry)
   {
      this.swingSplitFractions = swingSplitFractions;
      this.transferSplitFractions = transferSplitFractions;

      exitMultiplier = new DoubleYoVariable(namePrefix + name, registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      exitMultiplier.setToNaN();
   }

   public void compute(int numberOfFootstepsToConsider, List<DoubleYoVariable> doubleSupportDurations, List<DoubleYoVariable> singleSupportDurations,
         boolean useTwoCMPs, boolean isInTransfer, double omega0)
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
      exitMultiplier.set(computeStanceExitCMPRecursionOneCMP());
   }

   private void computeWithTwoCMPs(List<DoubleYoVariable> doubleSupportDurations, List<DoubleYoVariable> singleSupportDurations,
         boolean isInTransfer, double omega0)
   {
      double currentSwingOnExit = (1.0 - swingSplitFractions.get(0).getDoubleValue()) * singleSupportDurations.get(0).getDoubleValue();
      double nextTransferOnExit = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

      double currentTransferOnEntry, currentSwingOnEntry;

      if (isInTransfer)
      {
         currentTransferOnEntry = (1.0 - transferSplitFractions.get(0).getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();
         currentSwingOnEntry = swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();
      }
      else
      {
         currentTransferOnEntry = 0.0;
         currentSwingOnEntry = 0.0;
      }

      double timeSpentOnEntryCMP = currentTransferOnEntry + currentSwingOnEntry;
      double timeSpentOnExitCMP = currentSwingOnExit + nextTransferOnExit;

      double exitMultiplier = computeStanceExitCMPRecursionTwoCMPs(timeSpentOnEntryCMP, timeSpentOnExitCMP, omega0);
      this.exitMultiplier.set(exitMultiplier);
   }

   public static double computeStanceExitCMPRecursionOneCMP()
   {
      return 0.0;
   }

   public static double computeStanceExitCMPRecursionTwoCMPs(double currentTimeSpentOnEntryCMP, double currentTimeSpentOnExitCMP, double omega0)
   {
      return Math.exp(-omega0 * currentTimeSpentOnEntryCMP) * (1.0 - Math.exp(-omega0 * currentTimeSpentOnExitCMP));
   }

   public double getExitMultiplier()
   {
      return exitMultiplier.getDoubleValue();
   }
}
