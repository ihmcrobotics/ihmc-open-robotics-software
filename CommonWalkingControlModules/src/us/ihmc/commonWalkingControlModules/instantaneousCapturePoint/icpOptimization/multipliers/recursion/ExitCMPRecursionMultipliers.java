package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class ExitCMPRecursionMultipliers
{
   private static final String name = "CMPExitRecursionMultiplier";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<YoDouble> exitMultipliers = new ArrayList<>();

   private final List<YoDouble> swingSplitFractions;
   private final List<YoDouble> transferSplitFractions;

   private final int maximumNumberOfFootstepsToConsider;

   public ExitCMPRecursionMultipliers(String namePrefix, int maximumNumberOfFootstepsToConsider, List<YoDouble> swingSplitFractions,
         List<YoDouble> transferSplitFractions, YoVariableRegistry parentRegistry)
   {
      this.maximumNumberOfFootstepsToConsider = maximumNumberOfFootstepsToConsider;
      this.swingSplitFractions = swingSplitFractions;
      this.transferSplitFractions = transferSplitFractions;

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         YoDouble exitMultiplier = new YoDouble(namePrefix + name + i, registry);
         exitMultiplier.setToNaN();
         exitMultipliers.add(exitMultiplier);
      }

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
         exitMultipliers.get(i).setToNaN();
   }

   public void compute(int numberOfStepsToConsider, int numberOfStepsRegistered,
         List<YoDouble> doubleSupportDurations, List<YoDouble> singleSupportDurations, boolean useTwoCMPs, double omega0)
   {
      if (numberOfStepsToConsider > doubleSupportDurations.size())
         throw new RuntimeException("Double Support Durations list is not long enough");
      if (numberOfStepsToConsider > singleSupportDurations.size())
         throw new RuntimeException("Single Support Durations list is not long enough");

      if (useTwoCMPs)
         computeWithTwoCMPs(numberOfStepsToConsider, numberOfStepsRegistered, doubleSupportDurations, singleSupportDurations, omega0);
      else
         computeWithOneCMP(numberOfStepsToConsider, numberOfStepsRegistered);
   }

   private void computeWithOneCMP(int numberOfStepsToConsider, int numberOfStepsRegistered)
   {
      for (int i = 0; i < numberOfStepsToConsider; i++)
      {
         if (i == numberOfStepsRegistered)
            break;
         else
            exitMultipliers.get(i).set(computeExitRecursionMultiplierOneCMP());
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

         double exitRecursion = computeExitRecursionMultiplierTwoCMPs(recursionTime, timeSpentOnEntryCMP, timeSpentOnExitCMP, omega0);

         exitMultipliers.get(i - 1).set(exitRecursion);

         if (i >= numberOfStepsRegistered)
            break; // this is the final transfer

         recursionTime += timeSpentOnEntryCMP + timeSpentOnExitCMP;
      }
   }

   public static double computeExitRecursionMultiplierTwoCMPs(double totalRecursionTime, double timeSpentOnEntryCMP, double timeSpentOnExitCMP, double omega0)
   {
      return Math.exp(-omega0 * (totalRecursionTime + timeSpentOnEntryCMP)) * (1.0 - Math.exp(-omega0 * timeSpentOnExitCMP));
   }

   public static double computeExitRecursionMultiplierOneCMP()
   {
      return 0.0;
   }

   public double getExitMultiplier(int footstepIndex)
   {
      return exitMultipliers.get(footstepIndex).getDoubleValue();
   }
}
