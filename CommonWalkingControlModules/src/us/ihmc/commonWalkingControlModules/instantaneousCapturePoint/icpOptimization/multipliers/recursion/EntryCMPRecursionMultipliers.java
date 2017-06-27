package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class EntryCMPRecursionMultipliers
{
   private static final String name = "CMPEntryRecursionMultiplier";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<YoDouble> entryMultipliers = new ArrayList<>();

   private final List<YoDouble> swingSplitFractions;
   private final List<YoDouble> transferSplitFractions;

   private final int maximumNumberOfFootstepsToConsider;

   public EntryCMPRecursionMultipliers(String namePrefix, int maximumNumberOfFootstepsToConsider, List<YoDouble> swingSplitFractions,
         List<YoDouble> transferSplitFractions, YoVariableRegistry parentRegistry)
   {
      this.maximumNumberOfFootstepsToConsider = maximumNumberOfFootstepsToConsider;
      this.swingSplitFractions = swingSplitFractions;
      this.transferSplitFractions = transferSplitFractions;

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         YoDouble entryMultiplier = new YoDouble(namePrefix + name + i, registry);
         entryMultiplier.setToNaN();
         entryMultipliers.add(entryMultiplier);
      }

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
         entryMultipliers.get(i).setToNaN();
   }

   public void compute(int numberOfStepsToConsider, int numberOfStepsRegistered,
         List<YoDouble> doubleSupportDurations, List<YoDouble> singleSupportDurations,
         boolean useTwoCMPs, double omega0)
   {
      if (numberOfStepsToConsider > doubleSupportDurations.size())
         throw new RuntimeException("Double Support Durations list is not long enough");
      if (numberOfStepsToConsider > singleSupportDurations.size())
         throw new RuntimeException("Single Support Durations list is not long enough");

      if (useTwoCMPs)
         computeWithTwoCMPs(numberOfStepsToConsider, numberOfStepsRegistered, doubleSupportDurations, singleSupportDurations, omega0);
      else
         computeWithOneCMP(numberOfStepsToConsider, numberOfStepsRegistered, doubleSupportDurations, singleSupportDurations, omega0);
   }

   private void computeWithOneCMP(int numberOfStepsToConsider, int numberOfStepsRegistered,
         List<YoDouble> doubleSupportDurations, List<YoDouble> singleSupportDurations, double omega0)
   {
      double recursionTime = 0.0;
      for (int i = 1; i < numberOfStepsToConsider + 1; i++)
      {
         double currentTransferOnCMP, currentSwingOnCMP, nextTransferOnCMP;

         if (i < numberOfStepsRegistered)
         { // this is the next step
            currentTransferOnCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue();
            currentSwingOnCMP = singleSupportDurations.get(i).getDoubleValue();
            nextTransferOnCMP = transferSplitFractions.get(i + 1).getDoubleValue() * doubleSupportDurations.get(i + 1).getDoubleValue();
         }
         else
         { // this is the final transfer
            currentTransferOnCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue();
            currentSwingOnCMP = 0.0;
            nextTransferOnCMP = 0.0;
         }

         double timeOnCMP = currentTransferOnCMP + currentSwingOnCMP + nextTransferOnCMP;

         double entryRecursion = computeEntryRecursionMultiplierOneCMP(recursionTime, timeOnCMP, omega0);
         entryMultipliers.get(i - 1).set(entryRecursion);

         if (i >= numberOfStepsRegistered)
            break; // this is the final transfer

         recursionTime += timeOnCMP;
      }
   }

   private void computeWithTwoCMPs(int numberOfStepsToConsider, int numberOfStepsRegistered,
         List<YoDouble> doubleSupportDurations, List<YoDouble> singleSupportDurations, double omega0)
   {
      double recursionTime = 0.0;

      for (int i = 1; i < numberOfStepsToConsider + 1; i++)
      {
         double currentTransferOnEntryCMP, currentSwingOnEntryCMP, currentSwingOnExitCMP, nextTransferOnExitCMP;

         if (i < numberOfStepsRegistered)
         { // this is the next step
            currentTransferOnEntryCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue();
            currentSwingOnEntryCMP = swingSplitFractions.get(i).getDoubleValue() * singleSupportDurations.get(i).getDoubleValue();
            currentSwingOnExitCMP = (1.0 - swingSplitFractions.get(i).getDoubleValue()) * singleSupportDurations.get(i).getDoubleValue();
            nextTransferOnExitCMP = transferSplitFractions.get(i + 1).getDoubleValue() * doubleSupportDurations.get(i + 1).getDoubleValue();
         }
         else
         { // this is the final transfer

            currentTransferOnEntryCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue();
            currentSwingOnEntryCMP = 0.0;
            currentSwingOnExitCMP = 0.0;
            nextTransferOnExitCMP = 0.0;
         }

         double timeSpentOnEntryCMP = currentTransferOnEntryCMP + currentSwingOnEntryCMP;
         double timeSpentOnExitCMP = currentSwingOnExitCMP + nextTransferOnExitCMP;

         double entryRecursion = computeEntryRecursionMultiplierTwoCMPs(recursionTime, timeSpentOnEntryCMP, omega0);

         entryMultipliers.get(i - 1).set(entryRecursion);

         if (i >= numberOfStepsRegistered)
            break; // this is the final transfer

         recursionTime += timeSpentOnEntryCMP + timeSpentOnExitCMP;
      }
   }

   public static double computeEntryRecursionMultiplierTwoCMPs(double totalRecursionTime, double timeSpentOnEntryCMP, double omega0)
   {
      return Math.exp(-omega0 * totalRecursionTime) * (1.0 - Math.exp(-omega0 * timeSpentOnEntryCMP));
   }

   public static double computeEntryRecursionMultiplierOneCMP(double totalRecursionTime, double timeOnCMP, double omega0)
   {
      return Math.exp(-omega0 * totalRecursionTime) * (1.0 - Math.exp(-omega0 * timeOnCMP));
   }

   public double getEntryMultiplier(int footstepIndex)
   {
      return entryMultipliers.get(footstepIndex).getDoubleValue();
   }
}
