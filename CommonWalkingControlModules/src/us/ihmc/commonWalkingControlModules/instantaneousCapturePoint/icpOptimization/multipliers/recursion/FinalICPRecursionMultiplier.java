package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class FinalICPRecursionMultiplier extends DoubleYoVariable
{
   private static final String name = "FinalICPRecursionMultiplier";

   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   public FinalICPRecursionMultiplier(String namePrefix, DoubleYoVariable exitCMPDurationInPercentOfStepTime, YoVariableRegistry registry)
   {
      super(namePrefix + name, registry);

      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;
   }

   public void reset()
   {
      this.set(0.0);
   }

   public void compute(int numberOfStepsToConsider, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
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
         computeWithTwoCMPs(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, isInTransfer, omega0);
      else
         computeWithOneCMP(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, omega0);
   }

   private void computeWithOneCMP(int numberOfStepsToConsider, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double omega0)
   {
      double recursionTime = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();

      for (int i = 0; i < numberOfStepsToConsider; i++)
         recursionTime += singleSupportDurations.get(i + 1).getDoubleValue() + doubleSupportDurations.get(i + 1).getDoubleValue();

      double icpRecursion = Math.exp(-omega0 * recursionTime);
      this.set(icpRecursion);

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
         recursionTime += singleSupportDurations.get(i + 1).getDoubleValue() + doubleSupportDurations.get(i + 1).getDoubleValue();
      }

      double icpRecursion = Math.exp(-omega0 * recursionTime);
      this.set(icpRecursion);

   }
}

