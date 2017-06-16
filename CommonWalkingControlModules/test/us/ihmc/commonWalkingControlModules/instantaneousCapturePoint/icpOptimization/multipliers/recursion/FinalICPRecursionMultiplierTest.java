package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.Random;

public class FinalICPRecursionMultiplierTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneStepTwoCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoOmega = new YoDouble("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 2;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
      ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         YoDouble swingSplitFraction = new YoDouble("swingSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         swingSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
         swingSplitFractions.add(swingSplitFraction);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < stepsRegistered; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
            transferSplitFractions.get(i).set(0.8 * random.nextDouble());
            swingSplitFractions.get(i).set(0.8 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());
         transferSplitFractions.get(stepsRegistered).set(0.8 * random.nextDouble());

         boolean useTwoCMPs = true;

         finalICPRecursionMultiplier.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double nextTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue() +
               swingSplitFractions.get(1).getDoubleValue() * singleSupportDurations.get(1).getDoubleValue();
         double nextTimeSpentOnExitCMP = (1.0 - swingSplitFractions.get(1).getDoubleValue()) * singleSupportDurations.get(1).getDoubleValue() +
               transferSplitFractions.get(2).getDoubleValue() * doubleSupportDurations.get(2).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * (nextTimeSpentOnEntryCMP + nextTimeSpentOnExitCMP));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertEquals(finalICPMultiplier, recursionMultipliers.getFinalICPMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneStepTwoCMPCalculationFinalStep()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoOmega = new YoDouble("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 1;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
      ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         YoDouble swingSplitFraction = new YoDouble("swingSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         swingSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
         swingSplitFractions.add(swingSplitFraction);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < stepsRegistered; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
            transferSplitFractions.get(i).set(0.8 * random.nextDouble());
            swingSplitFractions.get(i).set(0.8 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());
         transferSplitFractions.get(stepsRegistered).set(0.8 * random.nextDouble());

         boolean useTwoCMPs = true;

         finalICPRecursionMultiplier.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double nextTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue();
         double nextTimeSpentOnExitCMP = 0.0;

         double finalICPMultiplier = Math.exp(-omega * nextTimeSpentOnEntryCMP);
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertEquals(finalICPMultiplier, recursionMultipliers.getFinalICPMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneStepOneCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoOmega = new YoDouble("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 2;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
      ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < stepsRegistered; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
            transferSplitFractions.get(i).set(0.8 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());
         transferSplitFractions.get(stepsRegistered).set(0.8 * random.nextDouble());

         boolean useTwoCMPs = false;

         finalICPRecursionMultiplier.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double timeOnNextCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue()
               + singleSupportDurations.get(1).getDoubleValue() + transferSplitFractions.get(2).getDoubleValue() * doubleSupportDurations.get(2).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * timeOnNextCMP);
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertEquals(finalICPMultiplier, recursionMultipliers.getFinalICPMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneStepOneCMPCalculationFinalStep()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoOmega = new YoDouble("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 1;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
      ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < stepsRegistered; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
            transferSplitFractions.get(i).set(0.8 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());
         transferSplitFractions.get(stepsRegistered).set(0.8 * random.nextDouble());

         boolean useTwoCMPs = false;

         finalICPRecursionMultiplier.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double timeOnNextCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * timeOnNextCMP);
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertEquals(finalICPMultiplier, recursionMultipliers.getFinalICPMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepOneStepRegisteredTwoCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoOmega = new YoDouble("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 1;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
      ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         YoDouble swingSplitFraction = new YoDouble("swingSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         swingSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
         swingSplitFractions.add(swingSplitFraction);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < stepsRegistered; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
            transferSplitFractions.get(i).set(0.8 * random.nextDouble());
            swingSplitFractions.get(i).set(0.8 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());
         transferSplitFractions.get(stepsRegistered).set(0.8 * random.nextDouble());

         boolean useTwoCMPs = true;

         finalICPRecursionMultiplier.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double nextTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * nextTimeSpentOnEntryCMP);

         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertEquals(finalICPMultiplier, recursionMultipliers.getFinalICPMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepTwoStepRegisteredTwoCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoOmega = new YoDouble("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 2;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
      ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         YoDouble swingSplitFraction = new YoDouble("swingSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         swingSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
         swingSplitFractions.add(swingSplitFraction);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < stepsRegistered; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
            transferSplitFractions.get(i).set(0.8 * random.nextDouble());
            swingSplitFractions.get(i).set(0.8 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());
         transferSplitFractions.get(stepsRegistered).set(0.8 * random.nextDouble());

         boolean useTwoCMPs = true;

         finalICPRecursionMultiplier.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double nextTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue() +
               swingSplitFractions.get(1).getDoubleValue() * singleSupportDurations.get(1).getDoubleValue();
         double nextTimeSpentOnExitCMP = (1.0 - swingSplitFractions.get(1).getDoubleValue()) * singleSupportDurations.get(1).getDoubleValue() +
               transferSplitFractions.get(2).getDoubleValue() * doubleSupportDurations.get(2).getDoubleValue();
         double nextNextTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(2).getDoubleValue()) * doubleSupportDurations.get(2).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * (nextTimeSpentOnEntryCMP + nextTimeSpentOnExitCMP + nextNextTimeSpentOnEntryCMP));

         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertEquals(finalICPMultiplier, recursionMultipliers.getFinalICPMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepThreeStepRegisteredTwoCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoOmega = new YoDouble("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 3;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
      ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         YoDouble swingSplitFraction = new YoDouble("swingSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         swingSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
         swingSplitFractions.add(swingSplitFraction);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < stepsRegistered; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
            transferSplitFractions.get(i).set(0.8 * random.nextDouble());
            swingSplitFractions.get(i).set(0.8 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());
         transferSplitFractions.get(stepsRegistered).set(0.8 * random.nextDouble());

         boolean useTwoCMPs = true;

         finalICPRecursionMultiplier.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double nextTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue() +
               swingSplitFractions.get(1).getDoubleValue() * singleSupportDurations.get(1).getDoubleValue();
         double nextTimeSpentOnExitCMP = (1.0 - swingSplitFractions.get(1).getDoubleValue()) * singleSupportDurations.get(1).getDoubleValue() +
               transferSplitFractions.get(2).getDoubleValue() * doubleSupportDurations.get(2).getDoubleValue();
         double nextNextTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(2).getDoubleValue()) * doubleSupportDurations.get(2).getDoubleValue()
               + swingSplitFractions.get(2).getDoubleValue() * singleSupportDurations.get(2).getDoubleValue();
         double nextNextTimeSpentOnExitCMP = (1.0 - swingSplitFractions.get(2).getDoubleValue()) * singleSupportDurations.get(2).getDoubleValue()
               + transferSplitFractions.get(3).getDoubleValue() * doubleSupportDurations.get(3).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * (nextTimeSpentOnEntryCMP + nextTimeSpentOnExitCMP +
               nextNextTimeSpentOnEntryCMP + nextNextTimeSpentOnExitCMP));

         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertEquals(finalICPMultiplier, recursionMultipliers.getFinalICPMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepOneStepRegisteredOneCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoOmega = new YoDouble("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 1;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
      ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < stepsRegistered; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
            transferSplitFractions.get(i).set(0.8 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());
         transferSplitFractions.get(stepsRegistered).set(0.8 * random.nextDouble());

         boolean useTwoCMPs = false;

         finalICPRecursionMultiplier.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double timeOnNextCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * timeOnNextCMP);

         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertEquals(finalICPMultiplier, recursionMultipliers.getFinalICPMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepTwoStepRegisteredOneCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoOmega = new YoDouble("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 2;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
      ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < stepsRegistered; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
            transferSplitFractions.get(i).set(0.8 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());
         transferSplitFractions.get(stepsRegistered).set(0.8 * random.nextDouble());

         boolean useTwoCMPs = false;

         finalICPRecursionMultiplier.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double timeOnNextCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue()
               + singleSupportDurations.get(1).getDoubleValue() + transferSplitFractions.get(2).getDoubleValue() * doubleSupportDurations.get(2).getDoubleValue();
         double timeOnNextNextCMP = (1.0 - transferSplitFractions.get(2).getDoubleValue()) * doubleSupportDurations.get(2).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * (timeOnNextCMP + timeOnNextNextCMP));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertEquals(finalICPMultiplier, recursionMultipliers.getFinalICPMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepThreeStepRegisteredOneCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoOmega = new YoDouble("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 3;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
      ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < stepsRegistered; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
            transferSplitFractions.get(i).set(0.8 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());
         transferSplitFractions.get(stepsRegistered).set(0.8 * random.nextDouble());

         boolean useTwoCMPs = false;

         finalICPRecursionMultiplier.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double timeOnNextCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue()
               + singleSupportDurations.get(1).getDoubleValue() + transferSplitFractions.get(2).getDoubleValue() * doubleSupportDurations.get(2).getDoubleValue();
         double timeOnNextNextCMP = (1.0 - transferSplitFractions.get(2).getDoubleValue()) * doubleSupportDurations.get(2).getDoubleValue()
               + singleSupportDurations.get(2).getDoubleValue() + transferSplitFractions.get(3).getDoubleValue() * doubleSupportDurations.get(3).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * (timeOnNextCMP + timeOnNextNextCMP));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertEquals(finalICPMultiplier, recursionMultipliers.getFinalICPMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testNStepTwoCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoOmega = new YoDouble("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
      ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         YoDouble swingSplitFraction = new YoDouble("swingSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         swingSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
         swingSplitFractions.add(swingSplitFraction);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int j = 1; j < maxSteps; j++)
      {
         for (int iter = 0; iter < iters; iter++)
         {
            for (int i = 0; i < maxSteps + 1; i++)
            {
               doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
               singleSupportDurations.get(i).set(5.0 * random.nextDouble());
               transferSplitFractions.get(i).set(0.8 * random.nextDouble());
               swingSplitFractions.get(i).set(0.8 * random.nextDouble());
            }

            boolean useTwoCMPs = true;

            finalICPRecursionMultiplier.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
            recursionMultipliers.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

            double recursionTime = 0.0;

            for (int i = 1; i < j + 1; i++)
            {
               double timeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue() +
                     swingSplitFractions.get(i).getDoubleValue() * singleSupportDurations.get(i).getDoubleValue();
               double timeSpentOnExitCMP = (1.0 - swingSplitFractions.get(i).getDoubleValue()) * singleSupportDurations.get(i).getDoubleValue() +
                     transferSplitFractions.get(i + 1).getDoubleValue() * doubleSupportDurations.get(i + 1).getDoubleValue();
               recursionTime += timeSpentOnEntryCMP + timeSpentOnExitCMP;
            }

            double finalICPMultiplier = Math.exp(-omega * recursionTime);
            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
            Assert.assertEquals(finalICPMultiplier, recursionMultipliers.getFinalICPMultiplier(), epsilon);
            Assert.assertFalse(Double.isNaN(finalICPMultiplier));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testNStepTwoCMPCalculationFinalTransfer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoOmega = new YoDouble("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
      ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         YoDouble swingSplitFraction = new YoDouble("swingSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         swingSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
         swingSplitFractions.add(swingSplitFraction);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int j = 1; j < maxSteps; j++)
      {
         for (int iter = 0; iter < iters; iter++)
         {
            for (int i = 0; i < maxSteps + 1; i++)
            {
               doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
               singleSupportDurations.get(i).set(5.0 * random.nextDouble());
               transferSplitFractions.get(i).set(0.8 * random.nextDouble());
               swingSplitFractions.get(i).set(0.8 * random.nextDouble());
            }

            boolean useTwoCMPs = true;

            finalICPRecursionMultiplier.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
            recursionMultipliers.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

            double recursionTime = 0.0;

            for (int i = 1; i < j + 1; i++)
            {
               boolean isLast = i == j;
               double timeSpentOnEntryCMP, timeSpentOnExitCMP;

               if (isLast)
               {
                  timeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue();
                  timeSpentOnExitCMP = 0.0;
               }
               else
               {
                  timeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue()
                        + swingSplitFractions.get(i).getDoubleValue() * singleSupportDurations.get(i).getDoubleValue();
                  timeSpentOnExitCMP = (1.0 - swingSplitFractions.get(i).getDoubleValue()) * singleSupportDurations.get(i).getDoubleValue()
                        + transferSplitFractions.get(i + 1).getDoubleValue() * doubleSupportDurations.get(i + 1).getDoubleValue();
               }

               recursionTime += timeSpentOnEntryCMP + timeSpentOnExitCMP;
            }

            double finalICPMultiplier = Math.exp(-omega * recursionTime);
            Assert.assertFalse(Double.isNaN(finalICPMultiplier));
            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
            Assert.assertEquals(finalICPMultiplier, recursionMultipliers.getFinalICPMultiplier(), epsilon);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testNStepOneCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoOmega = new YoDouble("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
      ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int j = 1; j < maxSteps; j++)
      {
         for (int iter = 0; iter < iters; iter++)
         {
            for (int i = 0; i < maxSteps + 1; i++)
            {
               doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
               singleSupportDurations.get(i).set(5.0 * random.nextDouble());
               transferSplitFractions.get(i).set(0.8 * random.nextDouble());
            }

            boolean useTwoCMPs = false;

            finalICPRecursionMultiplier.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
            recursionMultipliers.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

            double recursionTime = 0.0;
            for (int i = 1; i < j + 1; i++)
            {
               double timeOnCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue()
                     + singleSupportDurations.get(i).getDoubleValue() + transferSplitFractions.get(i + 1).getDoubleValue() * doubleSupportDurations.get(i + 1).getDoubleValue();
               recursionTime += timeOnCMP;
            }

            double finalICPMultiplier = Math.exp(-omega * recursionTime);
            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
            Assert.assertEquals(finalICPMultiplier, recursionMultipliers.getFinalICPMultiplier(), epsilon);
            Assert.assertFalse(Double.isNaN(finalICPMultiplier));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testNStepOneCMPCalculationFinalTransfer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoOmega = new YoDouble("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
      ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int j = 1; j < maxSteps; j++)
      {
         for (int iter = 0; iter < iters; iter++)
         {
            for (int i = 0; i < maxSteps + 1; i++)
            {
               doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
               singleSupportDurations.get(i).set(5.0 * random.nextDouble());
               transferSplitFractions.get(i).set(0.8 * random.nextDouble());
            }

            boolean useTwoCMPs = false;

            finalICPRecursionMultiplier.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
            recursionMultipliers.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

            double recursionTime = 0.0;
            for (int i = 1; i < j + 1; i++)
            {
               boolean isLast = i == j;
               double timeOnCMP;

               if (isLast)
                  timeOnCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue();
               else
                  timeOnCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue()
                        + singleSupportDurations.get(i).getDoubleValue() + transferSplitFractions.get(i + 1).getDoubleValue() * doubleSupportDurations.get(i + 1).getDoubleValue();

               recursionTime += timeOnCMP;
            }

            double finalICPMultiplier = Math.exp(-omega * recursionTime);
            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
            Assert.assertEquals(finalICPMultiplier, recursionMultipliers.getFinalICPMultiplier(), epsilon);
            Assert.assertFalse(Double.isNaN(finalICPMultiplier));
         }
      }
   }
}
