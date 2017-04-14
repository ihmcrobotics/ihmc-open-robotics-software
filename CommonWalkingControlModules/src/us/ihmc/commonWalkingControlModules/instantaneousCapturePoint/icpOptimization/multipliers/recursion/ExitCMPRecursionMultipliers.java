package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class ExitCMPRecursionMultipliers
{
   private static final String name = "CMPExitRecursionMultiplier";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<DoubleYoVariable> exitMultipliers = new ArrayList<>();

   private final List<DoubleYoVariable> swingSplitFractions;
   private final List<DoubleYoVariable> transferSplitFractions;

   private final int maximumNumberOfFootstepsToConsider;

   public ExitCMPRecursionMultipliers(String namePrefix, int maximumNumberOfFootstepsToConsider, List<DoubleYoVariable> swingSplitFractions,
         List<DoubleYoVariable> transferSplitFractions, YoVariableRegistry parentRegistry)
   {
      this.maximumNumberOfFootstepsToConsider = maximumNumberOfFootstepsToConsider;
      this.swingSplitFractions = swingSplitFractions;
      this.transferSplitFractions = transferSplitFractions;

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         DoubleYoVariable exitMultiplier = new DoubleYoVariable(namePrefix + name + i, registry);
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
         computeWithOneCMP(numberOfStepsToConsider, numberOfStepsRegistered);
   }

   private void computeWithOneCMP(int numberOfStepsToConsider, int numberOfStepsRegistered)
   {
      for (int i = 0; i < numberOfStepsToConsider; i++)
      {
         if (i == numberOfStepsRegistered)
            break;
         else
            exitMultipliers.get(i).set(0.0);
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

         double exitRecursion = Math.exp(-omega0 * (recursionTime + timeSpentOnEntryCMP)) * (1.0 - Math.exp(-omega0 * timeSpentOnExitCMP));

         exitMultipliers.get(i - 1).set(exitRecursion);

         if (i >= numberOfStepsRegistered)
            break; // this is the final transfer

         recursionTime += timeSpentOnEntryCMP + timeSpentOnExitCMP;
      }
   }

   public double getExitMultiplier(int footstepIndex)
   {
      return exitMultipliers.get(footstepIndex).getDoubleValue();
   }
}
