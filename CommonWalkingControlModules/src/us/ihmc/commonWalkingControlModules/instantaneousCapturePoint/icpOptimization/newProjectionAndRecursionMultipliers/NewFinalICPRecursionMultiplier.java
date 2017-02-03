package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.newProjectionAndRecursionMultipliers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class NewFinalICPRecursionMultiplier extends DoubleYoVariable
{
   private static final String name = "FinalICPRecursionMultiplier";

   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   public NewFinalICPRecursionMultiplier(String namePrefix, DoubleYoVariable exitCMPDurationInPercentOfStepTime, YoVariableRegistry registry)
   {
      super(namePrefix + name, registry);

      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;
   }

   public void compute(int numberOfStepsToConsider, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      if (numberOfStepsToConsider > doubleSupportDurations.size())
         throw new RuntimeException("Double Support Durations list is not long enough");
      if (numberOfStepsToConsider > singleSupportDurations.size())
         throw new RuntimeException("Single Support Durations list is not long enough");

      if (useTwoCMPs)
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
      else
      {
         // // TODO: 2/3/17  
      }
   }

   public void reset()
   {
      this.set(0.0);
   }
}
