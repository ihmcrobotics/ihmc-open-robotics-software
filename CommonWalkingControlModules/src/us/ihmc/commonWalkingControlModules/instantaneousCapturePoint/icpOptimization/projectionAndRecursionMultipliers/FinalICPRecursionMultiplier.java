package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class FinalICPRecursionMultiplier extends DoubleYoVariable
{
   private static final String namePrefix = "finalICPRecursionMultiplier";

   private final DoubleYoVariable doubleSupportSplitFraction;

   public FinalICPRecursionMultiplier(YoVariableRegistry registry, DoubleYoVariable doubleSupportSplitFraction)
   {
      this(namePrefix, registry, doubleSupportSplitFraction);
   }

   public FinalICPRecursionMultiplier(String name, YoVariableRegistry registry, DoubleYoVariable doubleSupportSplitFraction)
   {
      super(name, registry);

      if (doubleSupportSplitFraction == null)
         this.doubleSupportSplitFraction = new DoubleYoVariable(namePrefix + "_DoubleSupportSplitFraction", registry);
      else
         this.doubleSupportSplitFraction = doubleSupportSplitFraction;
   }

   public void compute(int numberOfStepsToConsider, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      if (numberOfStepsToConsider > doubleSupportDurations.size())
         throw new RuntimeException("Double Support Durations list is not long enough");
      if (numberOfStepsToConsider > singleSupportDurations.size())
         throw new RuntimeException("Single Support Durations list is not long enough");

      double timeToFinish;
      if (useTwoCMPs && isInTransfer)
      {
         double endOfDoubleSupport = doubleSupportSplitFraction.getDoubleValue() * doubleSupportDurations.get(0).getDoubleValue();
         timeToFinish = endOfDoubleSupport + doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
      }
      else
      {
         timeToFinish = doubleSupportSplitFraction.getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

         if (isInTransfer)
            timeToFinish += singleSupportDurations.get(0).getDoubleValue();
      }

      if (numberOfStepsToConsider == 0)
         timeToFinish = 0.0;

      double totalTimeForFinalICPRecursion = timeToFinish;
      for (int i = 0; i < numberOfStepsToConsider; i++)
      {
         double stepDuration = singleSupportDurations.get(i + 1).getDoubleValue() + doubleSupportDurations.get(i + 1).getDoubleValue();
         totalTimeForFinalICPRecursion += stepDuration;
      }

      this.set(Math.exp(-omega0 * totalTimeForFinalICPRecursion));
   }

   public void reset()
   {
      this.set(0.0);
   }
}
