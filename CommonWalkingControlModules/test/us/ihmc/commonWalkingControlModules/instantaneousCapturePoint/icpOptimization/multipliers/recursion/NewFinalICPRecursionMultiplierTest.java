package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import java.util.ArrayList;
import java.util.Random;

public class NewFinalICPRecursionMultiplierTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneStepTwoCMPCalculation()
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

      NewFinalICPRecursionMultiplier finalICPRecursionMultiplier = new NewFinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

         finalICPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double upcomingStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;

         double finalICPMultiplier = Math.exp(-omega * (currentTimeSpentOnExitCMP + upcomingStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);

         // setup for in transfer
         isInTransfer = true;
         finalICPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         finalICPMultiplier = Math.exp(-omega * (currentStepDuration + upcomingStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneStepOneCMPCalculation()
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

      NewFinalICPRecursionMultiplier finalICPRecursionMultiplier = new NewFinalICPRecursionMultiplier("", exitCMPRatio, registry);

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
         boolean useTwoCMPs = false;

         finalICPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double upcomingStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * (currentStepDuration + upcomingStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);

         // setup for in transfer
         isInTransfer = true;
         finalICPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoStepTwoCMPCalculation()
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

      NewFinalICPRecursionMultiplier finalICPRecursionMultiplier = new NewFinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

         finalICPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double nextNextStepDuration = doubleSupportDurations.get(2).getDoubleValue() + singleSupportDurations.get(2).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;

         double finalICPMultiplier = Math.exp(-omega * (currentTimeSpentOnExitCMP + nextStepDuration + nextNextStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);

         // setup for in transfer
         isInTransfer = true;
         finalICPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         finalICPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration + nextNextStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoStepOneCMPCalculation()
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

      NewFinalICPRecursionMultiplier finalICPRecursionMultiplier = new NewFinalICPRecursionMultiplier("", exitCMPRatio, registry);

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
         boolean useTwoCMPs = false;

         finalICPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double nextNextStepDuration = doubleSupportDurations.get(2).getDoubleValue() + singleSupportDurations.get(2).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration + nextNextStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);

         // setup for in transfer
         isInTransfer = true;
         finalICPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testNStepTwoCMPCalculation()
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

      NewFinalICPRecursionMultiplier finalICPRecursionMultiplier = new NewFinalICPRecursionMultiplier("", exitCMPRatio, registry);

      for (int j = 1; j < maxSteps; j++)
      {
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

            finalICPRecursionMultiplier.compute(j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
            double recursionTime = exitRatio * currentStepDuration;

            for (int i = 0; i < j; i++)
               recursionTime += doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();

            double finalICPMultiplier = Math.exp(-omega * recursionTime);
            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);

            // setup for in transfer
            isInTransfer = true;
            finalICPRecursionMultiplier.compute(j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            recursionTime = currentStepDuration;
            for (int i = 0; i < j; i++)
               recursionTime += doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();

            finalICPMultiplier = Math.exp(-omega * recursionTime);
            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testNStepOneCMPCalculation()
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

      NewFinalICPRecursionMultiplier finalICPRecursionMultiplier = new NewFinalICPRecursionMultiplier("", exitCMPRatio, registry);

      for (int j = 1; j < maxSteps; j++)
      {
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
            boolean useTwoCMPs = false;

            finalICPRecursionMultiplier.compute(j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
            double recursionTime = currentStepDuration;
            for (int i = 0; i < j; i++)
            {
               recursionTime += doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();
            }

            double finalICPMultiplier = Math.exp(-omega * recursionTime);
            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);

            // setup for in transfer
            isInTransfer = true;
            finalICPRecursionMultiplier.compute(j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         }
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
