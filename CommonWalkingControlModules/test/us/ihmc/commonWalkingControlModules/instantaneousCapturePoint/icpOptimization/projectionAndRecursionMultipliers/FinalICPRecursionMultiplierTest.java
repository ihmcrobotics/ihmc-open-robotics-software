package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.FinalICPRecursionMultiplier;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import java.util.ArrayList;
import java.util.Random;

public class FinalICPRecursionMultiplierTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable defaultDoubleSupportSplitFraction = new DoubleYoVariable("defaultDoubleSupportSplitFraction", registry);
      DoubleYoVariable upcomingDoubleSupportSplitFraction = new DoubleYoVariable("upcomingDoubleSupportSplitFraction", registry);
      double omega = 3.0;
      yoOmega.set(omega);

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier(registry, defaultDoubleSupportSplitFraction, upcomingDoubleSupportSplitFraction);
      Random random = new Random();

      int maxSteps = 5;
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      boolean useTwoCMPs;
      boolean isInTransfer;

      int iters = 100;
      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < maxSteps; i++)
         {
            String name = "number of steps = " + i;

            double splitFraction = 0.7 * random.nextDouble();
            defaultDoubleSupportSplitFraction.set(splitFraction);
            upcomingDoubleSupportSplitFraction.set(splitFraction);

            for (int step = 0; step < maxSteps; step++)
            {
               doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
               singleSupportDurations.get(step).set(5.0 * random.nextDouble());
            }

            useTwoCMPs = false;
            isInTransfer = true;


            double upcomingInitialDoubleSupport = splitFraction * doubleSupportDurations.get(1).getDoubleValue();
            double totalTime = singleSupportDurations.get(0).getDoubleValue() + upcomingInitialDoubleSupport;
            if (i == 0)
               totalTime = 0.0;
            for (int step = 0; step < i; step++)
            {
               totalTime += doubleSupportDurations.get(step + 1).getDoubleValue() + singleSupportDurations.get(step + 1).getDoubleValue();
            }

            double expected = Math.exp(-omega * totalTime);

            finalICPRecursionMultiplier.compute(i, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);
            Assert.assertEquals("one cmp in transfer , " + name, expected, finalICPRecursionMultiplier.getDoubleValue(), epsilon);

            isInTransfer = false;

            finalICPRecursionMultiplier.compute(i, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            totalTime = upcomingInitialDoubleSupport;
            if (i == 0)
               totalTime = 0.0;
            for (int step = 0; step < i; step++)
            {
               double stepDuration = doubleSupportDurations.get(step + 1).getDoubleValue() + singleSupportDurations.get(step + 1).getDoubleValue();
               totalTime += stepDuration;
            }

            expected = Math.exp(-omega * totalTime);

            Assert.assertEquals("one cmp in swing, " + name, expected, finalICPRecursionMultiplier.getDoubleValue(), epsilon);

            useTwoCMPs = true;
            isInTransfer = true;

            finalICPRecursionMultiplier.compute(i, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            double endOfDoubleSupportTime = splitFraction * doubleSupportDurations.get(0).getDoubleValue();
            totalTime = endOfDoubleSupportTime + doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
            if (i == 0)
               totalTime = 0.0;
            for (int step = 0; step < i; step++)
            {
               totalTime += doubleSupportDurations.get(step + 1).getDoubleValue() + singleSupportDurations.get(step + 1).getDoubleValue();
            }

            expected = Math.exp(-omega * totalTime);

            Assert.assertEquals("two cmp in transfer, " + name, expected, finalICPRecursionMultiplier.getDoubleValue(), epsilon);

            isInTransfer = false;

            finalICPRecursionMultiplier.compute(i, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            totalTime = upcomingInitialDoubleSupport;
            if (i == 0)
               totalTime = 0.0;
            for (int step = 0; step < i; step++)
            {
               totalTime += doubleSupportDurations.get(step + 1).getDoubleValue() + singleSupportDurations.get(step + 1).getDoubleValue();
            }

            expected = Math.exp(-omega * totalTime);

            Assert.assertEquals("two cmp in swing, " + name, expected, finalICPRecursionMultiplier.getDoubleValue(), epsilon);

            finalICPRecursionMultiplier.reset();
            Assert.assertEquals("reset ", 0.0, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         }
      }
   }
}
