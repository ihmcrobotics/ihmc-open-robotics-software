package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class ExitCMPRecursionMultipliers
{
   private static final String name = "CMPExitRecursionMultiplier";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<DoubleYoVariable> exitMultipliers = new ArrayList<>();
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   private final int maximumNumberOfFootstepsToConsider;

   public ExitCMPRecursionMultipliers(String namePrefix, int maximumNumberOfFootstepsToConsider, DoubleYoVariable exitCMPDurationInPercentOfStepTime,
         YoVariableRegistry parentRegistry)
   {
      this.maximumNumberOfFootstepsToConsider = maximumNumberOfFootstepsToConsider;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;

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
      double firstStepTime = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();

      double currentTimeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * firstStepTime;

      double recursionTime;

      if (isInTransfer)
         recursionTime = firstStepTime;
      else
         recursionTime = currentTimeSpentOnExitCMP;


      for (int i = 0; i < numberOfStepsToConsider; i++)
      {
         double steppingDuration;
         if (i + 1 < numberOfStepsRegistered)
         { // this is the next step
            steppingDuration = singleSupportDurations.get(i + 1).getDoubleValue() + doubleSupportDurations.get(i + 1).getDoubleValue();
         }
         else
         { // this is the final transfer
            steppingDuration = doubleSupportDurations.get(i + 1).getDoubleValue();
         }

         double timeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * steppingDuration;
         double timeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * steppingDuration;

         double exitRecursion = Math.exp(-omega0 * (recursionTime + timeSpentOnEntryCMP)) * (1.0 - Math.exp(-omega0 * timeSpentOnExitCMP));

         exitMultipliers.get(i).set(exitRecursion);

         if (i + 1 == numberOfStepsRegistered)
            break; // this is the final transfer

         recursionTime += steppingDuration;
      }
   }

   public double getExitMultiplier(int footstepIndex)
   {
      return exitMultipliers.get(footstepIndex).getDoubleValue();
   }
}
