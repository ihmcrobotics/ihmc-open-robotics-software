package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class FinalICPRecursionMultiplier extends DoubleYoVariable
{
   private static final String name = "FinalICPRecursionMultiplier";

   private final List<DoubleYoVariable> swingSplitFractions;
   private final List<DoubleYoVariable> transferSplitFractions;

   public FinalICPRecursionMultiplier(String namePrefix, List<DoubleYoVariable> swingSplitFractions, List<DoubleYoVariable> transferSplitFractions,
         YoVariableRegistry registry)
   {
      super(namePrefix + name, registry);

      this.swingSplitFractions = swingSplitFractions;
      this.transferSplitFractions = transferSplitFractions;
   }

   public void reset()
   {
      this.setToNaN();
   }

   public void compute(int numberOfStepsToConsider, int numberOfStepsRegistered,
         ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      if (numberOfStepsToConsider > doubleSupportDurations.size())
         throw new RuntimeException("Double Support Durations list is not long enough");
      if (numberOfStepsToConsider > singleSupportDurations.size())
         throw new RuntimeException("Single Support Durations list is not long enough");

      if (numberOfStepsToConsider == 0)
      {
         this.set(1.0);
         return;
      }

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

         recursionTime += currentTransferOnCMP + currentSwingOnCMP + nextTransferOnCMP;

         if (i >= numberOfStepsRegistered)
            break;
      }

      double icpRecursion = Math.exp(-omega0 * recursionTime);

      this.set(icpRecursion);
   }

   private void computeWithTwoCMPs(int numberOfStepsToConsider, int numberOfStepsRegistered,
         ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean isInTransfer, double omega0)
   {
      double timeSpentOnCurrentEntryCMP = (1.0 - transferSplitFractions.get(0).getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue() +
            swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();
      double timeSpentOnCurrentExitCMP = (1.0 - swingSplitFractions.get(0).getDoubleValue()) * singleSupportDurations.get(0).getDoubleValue() +
            transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

      double recursionTime = timeSpentOnCurrentExitCMP;

      if (isInTransfer)
         recursionTime += timeSpentOnCurrentEntryCMP;

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

         recursionTime += timeSpentOnEntryCMP + timeSpentOnExitCMP;

         if (i >= numberOfStepsRegistered)
            break;
      }

      double icpRecursion = Math.exp(-omega0 * recursionTime);
      this.set(icpRecursion);
   }
}

