package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class StanceEntryCMPRecursionMultiplierTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneStepTwoCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      ArrayList<DoubleYoVariable> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         DoubleYoVariable transferSplitFraction = new DoubleYoVariable("transferSplitFraction" + i, registry);
         DoubleYoVariable swingSplitFraction = new DoubleYoVariable("swingSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         swingSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
         swingSplitFractions.add(swingSplitFraction);
      }

      StanceEntryCMPRecursionMultiplier entryCMPRecursionMultiplier = new StanceEntryCMPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < maxSteps + 1; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
            transferSplitFractions.get(i).set(0.8 * random.nextDouble());
            swingSplitFractions.get(i).set(0.8 * random.nextDouble());
         }

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         entryCMPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);
         recursionMultipliers.compute(1, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(0).getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue() +
               swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();

         double entryCMPMultiplier = 0.0;
         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultiplier.getEntryMultiplier(), epsilon);
         Assert.assertEquals(entryCMPMultiplier, recursionMultipliers.getStanceEntryMultiplier(), epsilon);

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);
         recursionMultipliers.compute(1, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         entryCMPMultiplier = (1.0 - Math.exp(-omega * currentTimeSpentOnEntryCMP));
         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultiplier.getEntryMultiplier(), epsilon);
         Assert.assertEquals(entryCMPMultiplier, recursionMultipliers.getStanceEntryMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneStepOneCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      ArrayList<DoubleYoVariable> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         DoubleYoVariable transferSplitFraction = new DoubleYoVariable("transferSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
      }

      StanceEntryCMPRecursionMultiplier entryCMPRecursionMultiplier = new StanceEntryCMPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < maxSteps + 1; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
            transferSplitFractions.get(i).set(0.8 * random.nextDouble());
         }

         boolean isInTransfer = false;
         boolean useTwoCMPs = false;

         entryCMPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);
         recursionMultipliers.compute(1, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double timeOnCMP = (1.0 - transferSplitFractions.get(0).getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue()
               + transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

         double entryCMPMultiplier = 1.0 - Math.exp(-omega * timeOnCMP);
         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultiplier.getEntryMultiplier(), epsilon);
         Assert.assertEquals(entryCMPMultiplier, recursionMultipliers.getStanceEntryMultiplier(), epsilon);

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);
         recursionMultipliers.compute(1, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultiplier.getEntryMultiplier(), epsilon);
         Assert.assertEquals(entryCMPMultiplier, recursionMultipliers.getStanceEntryMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoStepTwoCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      ArrayList<DoubleYoVariable> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         DoubleYoVariable transferSplitFraction = new DoubleYoVariable("transferSplitFraction" + i, registry);
         DoubleYoVariable swingSplitFraction = new DoubleYoVariable("swingSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         swingSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
         swingSplitFractions.add(swingSplitFraction);
      }

      StanceEntryCMPRecursionMultiplier entryCMPRecursionMultiplier = new StanceEntryCMPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < maxSteps + 1; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
            transferSplitFractions.get(i).set(0.8 * random.nextDouble());
            swingSplitFractions.get(i).set(0.8 * random.nextDouble());
         }

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         entryCMPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);
         recursionMultipliers.compute(2, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentTimeSpentOnEntryCMP = (1.0 - transferSplitFractions.get(0).getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue() +
               swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();

         double entryCMPMultiplier = 0.0;
         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultiplier.getEntryMultiplier(), epsilon);
         Assert.assertEquals(entryCMPMultiplier, recursionMultipliers.getStanceEntryMultiplier(), epsilon);

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);
         recursionMultipliers.compute(2, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         entryCMPMultiplier = (1.0 - Math.exp(-omega * currentTimeSpentOnEntryCMP));
         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultiplier.getEntryMultiplier(), epsilon);
         Assert.assertEquals(entryCMPMultiplier, recursionMultipliers.getStanceEntryMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoStepOneCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      ArrayList<DoubleYoVariable> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         DoubleYoVariable transferSplitFraction = new DoubleYoVariable("transferSplitFraction" + i, registry);
         transferSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
      }

      StanceEntryCMPRecursionMultiplier entryCMPRecursionMultiplier = new StanceEntryCMPRecursionMultiplier("", swingSplitFractions, transferSplitFractions, registry);
      RecursionMultipliers recursionMultipliers = new RecursionMultipliers("", maxSteps, swingSplitFractions, transferSplitFractions, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < maxSteps + 1; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
            transferSplitFractions.get(i).set(0.8 * random.nextDouble());
         }

         boolean isInTransfer = false;
         boolean useTwoCMPs = false;

         entryCMPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);
         recursionMultipliers.compute(2, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double timeOnCMP = (1.0 - transferSplitFractions.get(0).getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue()
               + transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

         double entryCMPMultiplier = 1.0 - Math.exp(-omega * timeOnCMP);
         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultiplier.getEntryMultiplier(), epsilon);
         Assert.assertEquals(entryCMPMultiplier, recursionMultipliers.getStanceEntryMultiplier(), epsilon);

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);
         recursionMultipliers.compute(2, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultiplier.getEntryMultiplier(), epsilon);
         Assert.assertEquals(entryCMPMultiplier, recursionMultipliers.getStanceEntryMultiplier(), epsilon);
      }
   }
}
