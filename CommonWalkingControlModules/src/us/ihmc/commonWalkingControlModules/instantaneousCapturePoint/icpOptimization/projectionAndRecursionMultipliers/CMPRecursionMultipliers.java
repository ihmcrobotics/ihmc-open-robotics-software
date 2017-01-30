package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class CMPRecursionMultipliers
{
   private static final String entryName = "CMPEntryRecursionMultiplier";
   private static final String exitName = "CMPExitRecursionMultiplier";
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<DoubleYoVariable> exitMultipliers = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> entryMultipliers = new ArrayList<>();

   private final DoubleYoVariable doubleSupportSplitFraction;
   private final DoubleYoVariable upcomingDoubleSupportSplitFraction;
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   private final int maximumNumberOfFootstepsToConsider;

   public CMPRecursionMultipliers(String namePrefix, int maximumNumberOfFootstepsToConsider, DoubleYoVariable doubleSupportSplitFraction,
         DoubleYoVariable upcomingDoubleSupportSplitFraction, DoubleYoVariable exitCMPDurationInPercentOfStepTime, YoVariableRegistry parentRegistry)
   {
      this.maximumNumberOfFootstepsToConsider = maximumNumberOfFootstepsToConsider;
      this.doubleSupportSplitFraction = doubleSupportSplitFraction;
      this.upcomingDoubleSupportSplitFraction = upcomingDoubleSupportSplitFraction;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         entryMultipliers.add(new DoubleYoVariable(namePrefix + entryName + i, registry));
         exitMultipliers.add(new DoubleYoVariable(namePrefix + exitName + i, registry));
      }

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         entryMultipliers.get(i).set(0.0);
         exitMultipliers.get(i).set(0.);
      }
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
         computeWithOneCMP(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, isInTransfer, omega0);
   }

   private void computeWithOneCMP(int numberOfStepsToConsider, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean isInTransfer, double omega0)
   {
      double timeToFinish = upcomingDoubleSupportSplitFraction.getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

      if (isInTransfer)
         timeToFinish += singleSupportDurations.get(0).getDoubleValue();

      double recursionTime = timeToFinish;
      for (int i = 0; i < numberOfStepsToConsider; i++)
      {
         double steppingDuration = doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();
         double stepRecursion = 1.0 - Math.exp(-omega0 * steppingDuration);

         double previousStepDuration = 0.0;
         if (i > 0)
            previousStepDuration = doubleSupportDurations.get(i).getDoubleValue() + singleSupportDurations.get(i).getDoubleValue();

         recursionTime += previousStepDuration;
         exitMultipliers.get(i).set(Math.exp(-omega0 * recursionTime) * stepRecursion);
         entryMultipliers.get(i).set(0.0);
      }
   }

   private void computeWithTwoCMPs(int numberOfStepsToConsider, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean isInTransfer, double omega0)
   {
      double firstStepTime = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
      double timeSpentOnInitialDoubleSupportUpcoming = upcomingDoubleSupportSplitFraction.getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
      double timeSpentOnEndDoubleSupportCurrent = (1.0 - doubleSupportSplitFraction.getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();

      double timeToFinish;
      if (isInTransfer)
         timeToFinish = timeSpentOnInitialDoubleSupportUpcoming - timeSpentOnEndDoubleSupportCurrent + firstStepTime;
      else
         timeToFinish = timeSpentOnInitialDoubleSupportUpcoming;

      double recursionTime = timeToFinish;
      for (int i = 0; i < numberOfStepsToConsider; i++)
      {
         double steppingDuration = singleSupportDurations.get(i + 1).getDoubleValue() + doubleSupportDurations.get(i + 1).getDoubleValue();

         double previousStepDuration = 0.0;
         if (i > 0)
            previousStepDuration = doubleSupportDurations.get(i).getDoubleValue() + singleSupportDurations.get(i).getDoubleValue();

         recursionTime += previousStepDuration;

         double totalTimeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * steppingDuration;
         double totalTimeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * steppingDuration;

         double exitMultiplierTime = recursionTime + totalTimeSpentOnEntryCMP;
         double entryMultiplierTime = recursionTime;

         double exitRecursion = Math.exp(-omega0 * exitMultiplierTime) * (1.0 - Math.exp(-omega0 * totalTimeSpentOnExitCMP));
         double entryRecursion = Math.exp(-omega0 * entryMultiplierTime) * (1.0 - Math.exp(-omega0 * totalTimeSpentOnEntryCMP));

         entryMultipliers.get(i).set(entryRecursion);
         exitMultipliers.get(i).set(exitRecursion);
      }
   }

   public double getExitMultiplier(int footstepIndex)
   {
      return exitMultipliers.get(footstepIndex).getDoubleValue();
   }

   public double getEntryMultiplier(int footstepIndex)
   {
      return entryMultipliers.get(footstepIndex).getDoubleValue();
   }
}
