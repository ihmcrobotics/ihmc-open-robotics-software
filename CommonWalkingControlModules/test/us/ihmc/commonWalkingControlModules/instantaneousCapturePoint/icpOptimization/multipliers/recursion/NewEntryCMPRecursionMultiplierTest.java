package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.CMPRecursionMultipliers;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import java.util.ArrayList;
import java.util.Random;

public class NewEntryCMPRecursionMultiplierTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneStepCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      NewEntryCMPRecursionMultiplier entryCMPRecursionMultiplier = new NewEntryCMPRecursionMultiplier("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         entryCMPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double upcomingStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;
         double upcomingTimeSpentOnEntryCMP = (1 - exitRatio) * upcomingStepDuration;

         double exitCMPMultiplier = Math.exp(-omega * currentTimeSpentOnExitCMP) * (1.0 - Math.exp(-omega * upcomingTimeSpentOnEntryCMP));
         Assert.assertEquals(exitCMPMultiplier, entryCMPRecursionMultiplier.getEntryMultiplier(0), epsilon);

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         exitCMPMultiplier = Math.exp(-omega * currentStepDuration) * (1.0 - Math.exp(-omega * upcomingTimeSpentOnEntryCMP));
         Assert.assertEquals(exitCMPMultiplier, entryCMPRecursionMultiplier.getEntryMultiplier(0), epsilon);
      }
   }

   /*
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      NewEntryCMPRecursionMultiplier cmpRecursionMultipliers = new NewEntryCMPRecursionMultiplier("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < maxSteps; i++)
         {
            double splitFraction = 0.7 * random.nextDouble();
            double exitRatio = 0.7 * random.nextDouble();
            exitCMPRatio.set(exitRatio);

            for (int step = 0; step < maxSteps; step++)
            {
               doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
               singleSupportDurations.get(step).set(5.0 * random.nextDouble());
            }

            boolean useTwoCMPs = false;
            boolean isInTransfer = true;

            cmpRecursionMultipliers.compute(i, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            double upcomingInitialDoubleSupport = splitFraction * doubleSupportDurations.get(1).getDoubleValue();
            double totalTime = singleSupportDurations.get(0).getDoubleValue() + upcomingInitialDoubleSupport;

            for (int j = 0; j < i; j++)
            {
               double stepDuration = doubleSupportDurations.get(j + 1).getDoubleValue() + singleSupportDurations.get(j + 1).getDoubleValue();
               double multiplier = Math.exp(-omega * totalTime);
               double exitMultiplier = multiplier * (1.0 - Math.exp(-omega * stepDuration));

               Assert.assertEquals("stepNumber = " + i +  ", index = " + j, 0.0, cmpRecursionMultipliers.getEntryMultiplier(j), epsilon);

               totalTime += stepDuration;
            }

            for (int j = 0; j < i; j++)
            {
               cmpRecursionMultipliers.reset();
               Assert.assertEquals("", 0.0, cmpRecursionMultipliers.getEntryMultiplier(j), epsilon);
            }

            isInTransfer = false;

            cmpRecursionMultipliers.compute(i, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            totalTime = upcomingInitialDoubleSupport;

            for (int j = 0; j < i; j++)
            {
               double stepDuration = doubleSupportDurations.get(j + 1).getDoubleValue() + singleSupportDurations.get(j + 1).getDoubleValue();
               double multiplier = Math.exp(-omega * totalTime);
               double exitMultiplier = multiplier * (1.0 - Math.exp(-omega * stepDuration));

               Assert.assertEquals("stepNumber = " + i +  ", index = " + j, 0.0, cmpRecursionMultipliers.getEntryMultiplier(j), epsilon);

               totalTime += stepDuration;
            }

            useTwoCMPs = true;
            isInTransfer = true;

            cmpRecursionMultipliers.compute(i, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            double startStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
            double endOfDoubleSupport = (1.0 - splitFraction) * doubleSupportDurations.get(0).getDoubleValue();
            double entryTime = (1.0 - exitRatio) * startStepDuration;
            totalTime = upcomingInitialDoubleSupport - endOfDoubleSupport + startStepDuration;

            for (int j = 0; j < i; j++)
            {
               double stepDuration = doubleSupportDurations.get(j + 1).getDoubleValue() + singleSupportDurations.get(j + 1).getDoubleValue();
               double currentEntryTime = (1.0 - exitRatio) * stepDuration;
               double currentExitTime = exitRatio * stepDuration;

               double entryMultiplierTime = totalTime;
               double exitMultiplierTime = totalTime + currentEntryTime;
               double exitMultiplier = Math.exp(-omega * exitMultiplierTime) * (1.0 - Math.exp(-omega * currentExitTime));
               double entryMultiplier = Math.exp(-omega * entryMultiplierTime) * ( 1.0 - Math.exp(-omega * currentEntryTime));

               Assert.assertEquals("stepNumber = " + i +  ", index = " + j, entryMultiplier, cmpRecursionMultipliers.getEntryMultiplier(j), epsilon);

               totalTime += stepDuration;
            }

            isInTransfer = false;

            cmpRecursionMultipliers.compute(i, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            totalTime = upcomingInitialDoubleSupport;

            for (int j = 0; j < i; j++)
            {
               double stepDuration = doubleSupportDurations.get(j + 1).getDoubleValue() + singleSupportDurations.get(j + 1).getDoubleValue();
               double currentEntryTime = (1.0 - exitRatio) * stepDuration;
               double currentExitTime = exitRatio * stepDuration;

               double entryMultiplierTime = totalTime;
               double exitMultiplierTime = totalTime + currentEntryTime;
               double exitMultiplier = Math.exp(-omega * exitMultiplierTime) * (1.0 - Math.exp(-omega * currentExitTime));
               double entryMultiplier = Math.exp(-omega * entryMultiplierTime) * ( 1.0 - Math.exp(-omega * currentEntryTime));

               Assert.assertEquals("stepNumber = " + i +  ", index = " + j, entryMultiplier, cmpRecursionMultipliers.getEntryMultiplier(j), epsilon);

               totalTime += stepDuration;
            }
         }
      }
   }
   */
}
