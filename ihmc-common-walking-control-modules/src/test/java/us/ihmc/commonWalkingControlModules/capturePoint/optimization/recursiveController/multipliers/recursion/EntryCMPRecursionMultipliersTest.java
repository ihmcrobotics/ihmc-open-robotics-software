package us.ihmc.commonWalkingControlModules.capturePoint.optimization.recursiveController.multipliers.recursion;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.recursion.EntryCMPRecursionMultipliers;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.recursion.RecursionMultipliers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.Random;

public class EntryCMPRecursionMultipliersTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderOneStepOneRegisteredTwoCMPCalculation()
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

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions,
            registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("other", maxSteps, swingSplitFractions, transferSplitFractions, registry);

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

         entryCMPRecursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double timeSpentOnEntry = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue();
         double entryCMPMultiplier = 1.0 - Math.exp(-omega * timeSpentOnEntry);

         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(entryCMPMultiplier, recursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertFalse(Double.isNaN(entryCMPMultiplier));
         for (int i = 1; i < maxSteps; i++)
         {
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
            Assert.assertTrue(Double.isNaN(recursionMultipliers.getEntryMultiplier(i)));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderOneStepTwoRegisteredTwoCMPCalculation()
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

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);
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

         entryCMPRecursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double upcomingTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue() +
               swingSplitFractions.get(1).getDoubleValue() * singleSupportDurations.get(1).getDoubleValue();

         double exitCMPMultiplier = 1.0 - Math.exp(-omega * upcomingTimeSpentOnEntryCMP);

         Assert.assertEquals(exitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(exitCMPMultiplier, recursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPMultiplier));
         for (int i = 1; i < maxSteps; i++)
         {
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
            Assert.assertTrue(Double.isNaN(recursionMultipliers.getEntryMultiplier(i)));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderOneStepOneRegisteredOneCMPCalculation()
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

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);
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

         entryCMPRecursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double upcomingStepDuration = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue();

         double entryCMPMultiplier = 1.0 - Math.exp(-omega * upcomingStepDuration);

         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(entryCMPMultiplier, recursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertFalse(Double.isNaN(entryCMPMultiplier));
         for (int i = 1; i < maxSteps; i++)
         {
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
            Assert.assertTrue(Double.isNaN(recursionMultipliers.getEntryMultiplier(i)));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderOneStepTwoRegisteredOneCMPCalculation()
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

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);
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

         entryCMPRecursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double timeOnNextCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue()
               + singleSupportDurations.get(1).getDoubleValue()
               + transferSplitFractions.get(2).getDoubleValue() * doubleSupportDurations.get(2).getDoubleValue();

         double entryCMPMultiplier = 1.0 - Math.exp(-omega * timeOnNextCMP);

         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(entryCMPMultiplier, recursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertFalse(Double.isNaN(entryCMPMultiplier));
         for (int i = 1; i < maxSteps; i++)
         {
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
            Assert.assertTrue(Double.isNaN(recursionMultipliers.getEntryMultiplier(i)));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepsOneRegisteredTwoCMPCalculation()
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

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);
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

         entryCMPRecursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double upcomingTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue();

         double firstEntryCMPMultiplier = 1.0 - Math.exp(-omega * upcomingTimeSpentOnEntryCMP);


         Assert.assertEquals(firstEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(firstEntryCMPMultiplier, recursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertFalse(Double.isNaN(firstEntryCMPMultiplier));
         for (int i = 1; i < maxSteps; i++)
         {
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
            Assert.assertTrue(Double.isNaN(recursionMultipliers.getEntryMultiplier(i)));
         }

      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepsTwoRegisteredTwoCMPCalculation()
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

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);
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

         entryCMPRecursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double nextTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue() +
               swingSplitFractions.get(1).getDoubleValue() * singleSupportDurations.get(1).getDoubleValue();
         double nextTimeSpentOnExitCMP = (1.0 - swingSplitFractions.get(1).getDoubleValue()) * singleSupportDurations.get(1).getDoubleValue() +
               transferSplitFractions.get(2).getDoubleValue() * doubleSupportDurations.get(2).getDoubleValue();
         double nextNextTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(2).getDoubleValue()) * doubleSupportDurations.get(2).getDoubleValue();

         double firstExitCMPMultiplier = 1.0 - Math.exp(-omega * nextTimeSpentOnEntryCMP);
         double secondExitCMPMultiplier = Math.exp(-omega * (nextTimeSpentOnEntryCMP + nextTimeSpentOnExitCMP))
               * (1.0 - Math.exp(-omega * nextNextTimeSpentOnEntryCMP));

         Assert.assertEquals(firstExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertEquals(firstExitCMPMultiplier, recursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondExitCMPMultiplier, recursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstExitCMPMultiplier));
         Assert.assertFalse(Double.isNaN(secondExitCMPMultiplier));
         for (int i = 2; i < maxSteps; i++)
         {
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
            Assert.assertTrue(Double.isNaN(recursionMultipliers.getEntryMultiplier(i)));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepsThreeRegisteredTwoCMPCalculation()
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

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);
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

         entryCMPRecursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double nextTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue() +
               swingSplitFractions.get(1).getDoubleValue() * singleSupportDurations.get(1).getDoubleValue();
         double nextTimeSpentOnExitCMP = (1.0 - swingSplitFractions.get(1).getDoubleValue()) * singleSupportDurations.get(1).getDoubleValue() +
               transferSplitFractions.get(2).getDoubleValue() * doubleSupportDurations.get(2).getDoubleValue();
         double nextNextTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(2).getDoubleValue()) * doubleSupportDurations.get(2).getDoubleValue()
               + swingSplitFractions.get(2).getDoubleValue() * singleSupportDurations.get(2).getDoubleValue();

         double firstExitCMPMultiplier = 1.0 - Math.exp(-omega * nextTimeSpentOnEntryCMP);
         double secondExitCMPMultiplier = Math.exp(-omega * (nextTimeSpentOnEntryCMP + nextTimeSpentOnExitCMP))
               * (1.0 - Math.exp(-omega * nextNextTimeSpentOnEntryCMP));

         Assert.assertEquals(firstExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertEquals(firstExitCMPMultiplier, recursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondExitCMPMultiplier, recursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstExitCMPMultiplier));
         Assert.assertFalse(Double.isNaN(secondExitCMPMultiplier));
         for (int i = 2; i < maxSteps; i++)
         {
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
            Assert.assertTrue(Double.isNaN(recursionMultipliers.getEntryMultiplier(i)));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepsOneRegisteredOneCMPCalculation()
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

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);
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

         entryCMPRecursionMultipliers.compute(2, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(2, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double timeOnNextCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue();

         double firstEntryCMPMultiplier = (1.0 - Math.exp(-omega * timeOnNextCMP));

         Assert.assertEquals(firstEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(firstEntryCMPMultiplier, recursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertFalse(Double.isNaN(firstEntryCMPMultiplier));
         for (int i = 1; i < maxSteps; i++)
         {
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
            Assert.assertTrue(Double.isNaN(recursionMultipliers.getEntryMultiplier(i)));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepsTwoRegisteredOneCMPCalculation()
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

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);
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

         entryCMPRecursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double timeOnNextCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue() +
               singleSupportDurations.get(1).getDoubleValue() + transferSplitFractions.get(2).getDoubleValue() * doubleSupportDurations.get(2).getDoubleValue();
         double timeOnNextNextCMP = (1.0 - transferSplitFractions.get(2).getDoubleValue()) * doubleSupportDurations.get(2).getDoubleValue();

         double firstEntryCMPMultiplier = (1.0 - Math.exp(-omega * timeOnNextCMP));
         double secondEntryCMPMultiplier = Math.exp(-omega *  timeOnNextCMP) * (1.0 - Math.exp(-omega * timeOnNextNextCMP));

         Assert.assertEquals(firstEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertEquals(firstEntryCMPMultiplier, recursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondEntryCMPMultiplier, recursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstEntryCMPMultiplier));
         Assert.assertFalse(Double.isNaN(secondEntryCMPMultiplier));
         for (int i = 2; i < maxSteps; i++)
         {
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
            Assert.assertTrue(Double.isNaN(recursionMultipliers.getEntryMultiplier(i)));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepsThreeRegisteredOneCMPCalculation()
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

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);
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

         entryCMPRecursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
         recursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

         double timeOnNextCMP = (1.0 - transferSplitFractions.get(1).getDoubleValue()) * doubleSupportDurations.get(1).getDoubleValue() +
               singleSupportDurations.get(1).getDoubleValue() + transferSplitFractions.get(2).getDoubleValue() * doubleSupportDurations.get(2).getDoubleValue();
         double timeOnNextNextCMP = (1.0 - transferSplitFractions.get(2).getDoubleValue()) * doubleSupportDurations.get(2).getDoubleValue() +
               singleSupportDurations.get(2).getDoubleValue() + transferSplitFractions.get(3).getDoubleValue() * doubleSupportDurations.get(3).getDoubleValue();

         double firstEntryCMPMultiplier = 1.0 - Math.exp(-omega * timeOnNextCMP);
         double secondEntryCMPMultiplier = Math.exp(-omega *  timeOnNextCMP) * (1.0 - Math.exp(-omega * timeOnNextNextCMP));

         Assert.assertEquals(firstEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertEquals(firstEntryCMPMultiplier, recursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondEntryCMPMultiplier, recursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstEntryCMPMultiplier));
         Assert.assertFalse(Double.isNaN(secondEntryCMPMultiplier));
         for (int i = 2; i < maxSteps; i++)
         {
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
            Assert.assertTrue(Double.isNaN(recursionMultipliers.getEntryMultiplier(i)));
         }
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

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int j = 1; j < maxSteps; j++)
      {
         for (int iter = 0; iter < iters; iter++)
         {
            for (int step = 0; step < maxSteps; step++)
            {
               doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
               singleSupportDurations.get(step).set(5.0 * random.nextDouble());
               transferSplitFractions.get(step).set(0.8 * random.nextDouble());
               swingSplitFractions.get(step).set(0.8 * random.nextDouble());
            }

            boolean useTwoCMPs = true;

            entryCMPRecursionMultipliers.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
            recursionMultipliers.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

            double recursionTime = 0.0;
            for (int i = 1; i < j + 1; i++)
            {
               double timeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue() +
                     swingSplitFractions.get(i).getDoubleValue() * singleSupportDurations.get(i).getDoubleValue();
               double timeSpentOnExitCMP = (1.0 - swingSplitFractions.get(i).getDoubleValue()) * singleSupportDurations.get(i).getDoubleValue() +
                     transferSplitFractions.get(i + 1).getDoubleValue() * doubleSupportDurations.get(i + 1).getDoubleValue();
               double entryMultiplier = Math.exp(-omega * recursionTime) * ( 1.0 - Math.exp(-omega * timeSpentOnEntryCMP));

               Assert.assertEquals(entryMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(i - 1), epsilon);
               Assert.assertEquals(entryMultiplier, recursionMultipliers.getEntryMultiplier(i - 1), epsilon);

               recursionTime += timeSpentOnEntryCMP + timeSpentOnExitCMP;
            }
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

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int j = 1; j < maxSteps; j++)
      {
         for (int iter = 0; iter < iters; iter++)
         {
            for (int step = 0; step < maxSteps; step++)
            {
               doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
               singleSupportDurations.get(step).set(5.0 * random.nextDouble());
               transferSplitFractions.get(step).set(0.8 * random.nextDouble());
               swingSplitFractions.get(step).set(0.8 * random.nextDouble());
            }

            boolean useTwoCMPs = true;

            entryCMPRecursionMultipliers.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
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


               double entryMultiplier = Math.exp(-omega * recursionTime) * ( 1.0 - Math.exp(-omega * timeSpentOnEntryCMP));

               Assert.assertEquals(entryMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(i - 1), epsilon);
               Assert.assertEquals(entryMultiplier, recursionMultipliers.getEntryMultiplier(i - 1), epsilon);

               recursionTime += timeSpentOnEntryCMP + timeSpentOnExitCMP;
            }
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

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int j = 1; j < maxSteps; j++)
      {
         for (int iter = 0; iter < iters; iter++)
         {
            for (int step = 0; step < maxSteps; step++)
            {
               doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
               singleSupportDurations.get(step).set(5.0 * random.nextDouble());
               transferSplitFractions.get(step).set(0.8 * random.nextDouble());
            }

            boolean useTwoCMPs = false;

            entryCMPRecursionMultipliers.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
            recursionMultipliers.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

            double recursionTime = 0.0;
            for (int i = 1; i < j + 1; i++)
            {
               double timeSpentOnCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue() +
                     singleSupportDurations.get(i).getDoubleValue() + transferSplitFractions.get(i + 1).getDoubleValue() * doubleSupportDurations.get(i + 1).getDoubleValue();

               double entryMultiplier = Math.exp(-omega * recursionTime) * (1.0 - Math.exp(-omega * timeSpentOnCMP));

               Assert.assertEquals(entryMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(i - 1), epsilon);
               Assert.assertEquals(entryMultiplier, recursionMultipliers.getEntryMultiplier(i - 1), epsilon);

               recursionTime += timeSpentOnCMP;
            }
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testNStepOneCMPCalculationInTransfer()
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

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int j = 1; j < maxSteps; j++)
      {
         for (int iter = 0; iter < iters; iter++)
         {
            for (int step = 0; step < maxSteps; step++)
            {
               doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
               singleSupportDurations.get(step).set(5.0 * random.nextDouble());
               transferSplitFractions.get(step).set(0.8 * random.nextDouble());
            }

            boolean useTwoCMPs = false;

            entryCMPRecursionMultipliers.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);
            recursionMultipliers.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, omega);

            double recursionTime = 0.0;
            for (int i = 1; i < j + 1; i++)
            {
               boolean isLastStep = i == j;
               double timeSpentOnCMP;
               if (isLastStep)
                  timeSpentOnCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue();
               else
                  timeSpentOnCMP = (1.0 - transferSplitFractions.get(i).getDoubleValue()) * doubleSupportDurations.get(i).getDoubleValue() +
                        singleSupportDurations.get(i).getDoubleValue() + transferSplitFractions.get(i + 1).getDoubleValue() * doubleSupportDurations.get(i + 1).getDoubleValue();

               double entryMultiplier = Math.exp(-omega * recursionTime) * (1.0 - Math.exp(-omega * timeSpentOnCMP));

               Assert.assertEquals(entryMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(i - 1), epsilon);
               Assert.assertEquals(entryMultiplier, recursionMultipliers.getEntryMultiplier(i - 1), epsilon);

               recursionTime += timeSpentOnCMP;
            }
         }
      }
   }
}

