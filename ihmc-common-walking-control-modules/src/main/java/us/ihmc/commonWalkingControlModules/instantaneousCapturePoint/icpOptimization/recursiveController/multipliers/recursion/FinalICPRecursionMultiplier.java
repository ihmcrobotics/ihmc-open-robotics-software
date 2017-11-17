package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.recursion;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class FinalICPRecursionMultiplier extends YoDouble
{
   private static final String name = "FinalICPRecursionMultiplier";

   private final List<YoDouble> swingSplitFractions;
   private final List<YoDouble> transferSplitFractions;

   public FinalICPRecursionMultiplier(String namePrefix, List<YoDouble> swingSplitFractions, List<YoDouble> transferSplitFractions,
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
         List<YoDouble> doubleSupportDurations, List<YoDouble> singleSupportDurations,
         boolean useTwoCMPs, double omega0)
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
         computeWithTwoCMPs(numberOfStepsToConsider, numberOfStepsRegistered, doubleSupportDurations, singleSupportDurations, omega0);
      else
         computeWithOneCMP(numberOfStepsToConsider, numberOfStepsRegistered, doubleSupportDurations, singleSupportDurations, omega0);
   }

   private void computeWithOneCMP(int numberOfStepsToConsider, int numberOfStepsRegistered,
         List<YoDouble> doubleSupportDurations, List<YoDouble> singleSupportDurations,
         double omega0)
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

         recursionTime += currentTransferOnCMP + currentSwingOnCMP + nextTransferOnCMP;

         if (i >= numberOfStepsRegistered)
            break;
      }

      double icpRecursion = computeFinalICPRecursionMultiplier(recursionTime, omega0);
      this.set(icpRecursion);
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

         recursionTime += timeSpentOnEntryCMP + timeSpentOnExitCMP;

         if (i >= numberOfStepsRegistered)
            break;
      }

      double icpRecursion = computeFinalICPRecursionMultiplier(recursionTime, omega0);
      this.set(icpRecursion);
   }

   public static double computeFinalICPRecursionMultiplier(double totalRecursionTime, double omega0)
   {
      return Math.exp(-omega0 * totalRecursionTime);
   }
}

