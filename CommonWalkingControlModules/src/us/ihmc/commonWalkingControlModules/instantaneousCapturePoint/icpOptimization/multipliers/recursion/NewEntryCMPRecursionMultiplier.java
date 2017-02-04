package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class NewEntryCMPRecursionMultiplier
{
   private static final String name = "CMPEntryRecursionMultiplier";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<DoubleYoVariable> entryMultipliers = new ArrayList<>();
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   private final int maximumNumberOfFootstepsToConsider;

   public NewEntryCMPRecursionMultiplier(String namePrefix, int maximumNumberOfFootstepsToConsider, DoubleYoVariable exitCMPDurationInPercentOfStepTime,
         YoVariableRegistry parentRegistry)
   {
      this.maximumNumberOfFootstepsToConsider = maximumNumberOfFootstepsToConsider;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
         entryMultipliers.add(new DoubleYoVariable(namePrefix + name + i, registry));

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
         entryMultipliers.get(i).set(0.0);
   }

   public void compute(int numberOfStepsToConsider, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      if (numberOfStepsToConsider > doubleSupportDurations.size())
         throw new RuntimeException("Double Support Durations list is not long enough");
      if (numberOfStepsToConsider > singleSupportDurations.size())
         throw new RuntimeException("Single Support Durations list is not long enough");

      if (useTwoCMPs)
         computeWithTwoCMPs(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, isInTransfer, omega0);
      else
         computeWithOneCMP(numberOfStepsToConsider);
   }

   private void computeWithOneCMP(int numberOfStepsToConsider)
   {
      for (int i = 0; i < numberOfStepsToConsider; i++)
      {
         entryMultipliers.get(i).set(0.0);
      }
   }

   private void computeWithTwoCMPs(int numberOfStepsToConsider, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean isInTransfer, double omega0)
   {
      double firstStepTime = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();

      double currentTimeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * firstStepTime;

      double recursionTime;

      if (isInTransfer)
         recursionTime = firstStepTime;
      else
         recursionTime = currentTimeSpentOnExitCMP;


      for (int i = 0; i < numberOfStepsToConsider; i++)
      {
         double steppingDuration = singleSupportDurations.get(i + 1).getDoubleValue() + doubleSupportDurations.get(i + 1).getDoubleValue();
         double previousStepDuration = singleSupportDurations.get(i).getDoubleValue() + doubleSupportDurations.get(0).getDoubleValue();

         double timeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * steppingDuration;

         if (i > 0)
            recursionTime += previousStepDuration;

         double entryRecursion = Math.exp(-omega0 * recursionTime) * (1.0 - Math.exp(-omega0 * timeSpentOnEntryCMP));

         entryMultipliers.get(i).set(entryRecursion);
      }
   }

   public double getEntryMultiplier(int footstepIndex)
   {
      return entryMultipliers.get(footstepIndex).getDoubleValue();
   }
}
