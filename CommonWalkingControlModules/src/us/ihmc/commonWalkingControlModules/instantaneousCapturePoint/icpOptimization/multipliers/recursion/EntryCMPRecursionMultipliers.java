package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class EntryCMPRecursionMultipliers
{
   private static final String name = "CMPEntryRecursionMultiplier";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<DoubleYoVariable> entryMultipliers = new ArrayList<>();

   private final List<DoubleYoVariable> swingSplitFractions;
   private final List<DoubleYoVariable> transferSplitFractions;

   private final int maximumNumberOfFootstepsToConsider;

   public EntryCMPRecursionMultipliers(String namePrefix, int maximumNumberOfFootstepsToConsider, List<DoubleYoVariable> swingSplitFractions,
         List<DoubleYoVariable> transferSplitFractions, YoVariableRegistry parentRegistry)
   {
      this.maximumNumberOfFootstepsToConsider = maximumNumberOfFootstepsToConsider;
      this.swingSplitFractions = swingSplitFractions;
      this.transferSplitFractions = transferSplitFractions;

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         DoubleYoVariable entryMultiplier = new DoubleYoVariable(namePrefix + name + i, registry);
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
         ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      if (numberOfStepsToConsider > doubleSupportDurations.size())
         throw new RuntimeException("Double Support Durations list is not long enough");
      if (numberOfStepsToConsider > singleSupportDurations.size())
         throw new RuntimeException("Single Support Durations list is not long enough");

      if (useTwoCMPs)
         computeWithTwoCMPs(numberOfStepsToConsider, numberOfStepsRegistered, doubleSupportDurations, singleSupportDurations, isInTransfer, omega0);
      else
         computeWithOneCMP(numberOfStepsToConsider, numberOfStepsRegistered, doubleSupportDurations, singleSupportDurations, omega0);
   }

   private void computeWithOneCMP(int numberOfStepsToConsider, int numberOfStepsRegistered,
         ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double omega0)
   {
      double timeSpentOnCurrentCMP = (1.0 - transferSplitFractions.get(0).getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue() +
            singleSupportDurations.get(0).getDoubleValue() + transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
      double recursionTime = timeSpentOnCurrentCMP;

      for (int i = 1; i < numberOfStepsToConsider + 1; i++)
      {
         double steppingDuration;
         if (i < numberOfStepsRegistered)
         { // this is the next step
            steppingDuration = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue() +
                  singleSupportDurations.get(i).getDoubleValue() +
                  transferSplitFractions.get(i + 1).getDoubleValue() * doubleSupportDurations.get(i + 1).getDoubleValue();
         }
         else
         { // this is the final transfer
            steppingDuration = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue();
         }

         double entryRecursion = Math.exp(-omega0 * recursionTime) * (1.0 - Math.exp(-omega0 * steppingDuration));
         entryMultipliers.get(i - 1).set(entryRecursion);

         if (i >= numberOfStepsRegistered)
            break; // this is the final transfer

         recursionTime += steppingDuration;
      }
   }

   private void computeWithTwoCMPs(int numberOfStepsToConsider, int numberOfStepsRegistered,
         ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean isInTransfer, double omega0)
   {
      double currentTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(0).getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue() +
            swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();
      double currentTimeSpentOnExitCMP = (1.0 - swingSplitFractions.get(0).getDoubleValue()) * singleSupportDurations.get(0).getDoubleValue() +
            transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

      double recursionTime = currentTimeSpentOnExitCMP;

      if (isInTransfer)
         recursionTime += currentTimeSpentOnEntryCMP;


      for (int i = 1; i < numberOfStepsToConsider + 1; i++)
      {
         double timeSpentOnEntryCMP, timeSpentOnExitCMP;
         if (i < numberOfStepsRegistered)
         { // this is the next step
            double currentTransferOnEntryCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue();
            double currentSwingOnEntryCMP = swingSplitFractions.get(i).getDoubleValue() * singleSupportDurations.get(i).getDoubleValue();
            double currentSwingOnExitCMP = (1.0 - swingSplitFractions.get(i).getDoubleValue()) * singleSupportDurations.get(i).getDoubleValue();
            double nextTransferOnExitCMP = transferSplitFractions.get(i + 1).getDoubleValue() * doubleSupportDurations.get(i + 1).getDoubleValue();

            timeSpentOnEntryCMP = currentTransferOnEntryCMP + currentSwingOnEntryCMP;
            timeSpentOnExitCMP = currentSwingOnExitCMP + nextTransferOnExitCMP;
         }
         else
         { // this is the final transfer
            timeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue();
            timeSpentOnExitCMP = 0.0;
         }

         double entryRecursion = Math.exp(-omega0 * recursionTime) * (1.0 - Math.exp(-omega0 * timeSpentOnEntryCMP));

         entryMultipliers.get(i - 1).set(entryRecursion);

         if (i >= numberOfStepsRegistered)
            break; // this is the final transfer

         recursionTime += timeSpentOnEntryCMP + timeSpentOnExitCMP;
      }
   }

   public double getEntryMultiplier(int footstepIndex)
   {
      return entryMultipliers.get(footstepIndex).getDoubleValue();
   }
}
