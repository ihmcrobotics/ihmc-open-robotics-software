package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class EntryCMPRecursionMultipliersTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderOneStepOneRegisteredTwoCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 1;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int i = 0; i < stepsRegistered; i++)
         {
            doubleSupportDurations.get(i).set(2.0 * random.nextDouble());
            singleSupportDurations.get(i).set(5.0 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         entryCMPRecursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double upcomingStepDuration = doubleSupportDurations.get(1).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;
         double upcomingTimeSpentOnEntryCMP = (1 - exitRatio) * upcomingStepDuration;

         double exitCMPMultiplier = Math.exp(-omega * currentTimeSpentOnExitCMP) * (1.0 - Math.exp(-omega * upcomingTimeSpentOnEntryCMP));
         Assert.assertEquals(exitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPMultiplier));
         for (int i = 0; i < maxSteps; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         exitCMPMultiplier = Math.exp(-omega * currentStepDuration) * (1.0 - Math.exp(-omega * upcomingTimeSpentOnEntryCMP));
         Assert.assertEquals(exitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPMultiplier));
         for (int i = 0; i < maxSteps; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderOneStepTwoRegisteredTwoCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 2;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < stepsRegistered; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         entryCMPRecursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double upcomingStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;
         double upcomingTimeSpentOnEntryCMP = (1 - exitRatio) * upcomingStepDuration;

         double exitCMPMultiplier = Math.exp(-omega * currentTimeSpentOnExitCMP) * (1.0 - Math.exp(-omega * upcomingTimeSpentOnEntryCMP));
         Assert.assertEquals(exitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPMultiplier));
         for (int i = 0; i < stepsRegistered; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         exitCMPMultiplier = Math.exp(-omega * currentStepDuration) * (1.0 - Math.exp(-omega * upcomingTimeSpentOnEntryCMP));
         Assert.assertEquals(exitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPMultiplier));
         for (int i = 0; i < stepsRegistered; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderOneStepOneRegisteredOneCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 1;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < stepsRegistered; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());

         boolean isInTransfer = false;
         boolean useTwoCMPs = false;

         entryCMPRecursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double upcomingStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();

         double entryCMPMultiplier = Math.exp(-omega * currentStepDuration) * (1.0 - Math.exp(-omega * upcomingStepDuration));
         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertFalse(Double.isNaN(entryCMPMultiplier));
         for (int i = 0; i < stepsRegistered; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertFalse(Double.isNaN(entryCMPMultiplier));
         for (int i = 0; i < stepsRegistered; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderOneStepTwoRegisteredOneCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 2;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < stepsRegistered; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());

         boolean isInTransfer = false;
         boolean useTwoCMPs = false;

         entryCMPRecursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double upcomingStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();

         double entryCMPMultiplier = Math.exp(-omega * currentStepDuration) * (1.0 - Math.exp(-omega * upcomingStepDuration));
         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertFalse(Double.isNaN(entryCMPMultiplier));
         for (int i = 0; i < stepsRegistered; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultipliers.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(entryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertFalse(Double.isNaN(entryCMPMultiplier));
         for (int i = 0; i < stepsRegistered; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepsOneRegisteredTwoCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 1;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < stepsRegistered; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         entryCMPRecursionMultipliers.compute(2, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double nextNextStepDuration = doubleSupportDurations.get(2).getDoubleValue() + singleSupportDurations.get(2).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;
         double upcomingTimeSpentOnEntryCMP = (1 - exitRatio) * nextStepDuration;
         double nextNextTimeSpentOnEntryCMP = (1 - exitRatio) * nextNextStepDuration;

         double firstExitCMPMultiplier = Math.exp(-omega * currentTimeSpentOnExitCMP) * (1.0 - Math.exp(-omega * upcomingTimeSpentOnEntryCMP));
         double secondExitCMPMultiplier = Math.exp(-omega * (currentTimeSpentOnExitCMP + nextStepDuration)) * (1.0 - Math.exp(-omega * nextNextTimeSpentOnEntryCMP));

         Assert.assertEquals(firstExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstExitCMPMultiplier));
         for (int i = 1; i < maxSteps; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultipliers.compute(2, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         firstExitCMPMultiplier = Math.exp(-omega * currentStepDuration) * (1.0 - Math.exp(-omega * upcomingTimeSpentOnEntryCMP));
         secondExitCMPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration)) * (1.0 - Math.exp(-omega * nextNextTimeSpentOnEntryCMP));

         Assert.assertEquals(firstExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstExitCMPMultiplier));
         for (int i = 1; i < maxSteps; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepsTwoRegisteredTwoCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 2;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < stepsRegistered; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         entryCMPRecursionMultipliers.compute(2, 2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double nextNextStepDuration = doubleSupportDurations.get(2).getDoubleValue() + singleSupportDurations.get(2).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;
         double upcomingTimeSpentOnEntryCMP = (1 - exitRatio) * nextStepDuration;
         double nextNextTimeSpentOnEntryCMP = (1 - exitRatio) * nextNextStepDuration;

         double firstExitCMPMultiplier = Math.exp(-omega * currentTimeSpentOnExitCMP) * (1.0 - Math.exp(-omega * upcomingTimeSpentOnEntryCMP));
         double secondExitCMPMultiplier = Math.exp(-omega * (currentTimeSpentOnExitCMP + nextStepDuration)) * (1.0 - Math.exp(-omega * nextNextTimeSpentOnEntryCMP));

         Assert.assertEquals(firstExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstExitCMPMultiplier));
         Assert.assertFalse(Double.isNaN(secondExitCMPMultiplier));
         for (int i = 2; i < maxSteps; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultipliers.compute(2, 2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         firstExitCMPMultiplier = Math.exp(-omega * currentStepDuration) * (1.0 - Math.exp(-omega * upcomingTimeSpentOnEntryCMP));
         secondExitCMPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration)) * (1.0 - Math.exp(-omega * nextNextTimeSpentOnEntryCMP));

         Assert.assertEquals(firstExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstExitCMPMultiplier));
         Assert.assertFalse(Double.isNaN(secondExitCMPMultiplier));
         for (int i = 2; i < maxSteps; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepsThreeRegisteredTwoCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 3;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < stepsRegistered; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         entryCMPRecursionMultipliers.compute(2, 3, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double nextNextStepDuration = doubleSupportDurations.get(2).getDoubleValue() + singleSupportDurations.get(2).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;
         double upcomingTimeSpentOnEntryCMP = (1 - exitRatio) * nextStepDuration;
         double nextNextTimeSpentOnEntryCMP = (1 - exitRatio) * nextNextStepDuration;

         double firstExitCMPMultiplier = Math.exp(-omega * currentTimeSpentOnExitCMP) * (1.0 - Math.exp(-omega * upcomingTimeSpentOnEntryCMP));
         double secondExitCMPMultiplier = Math.exp(-omega * (currentTimeSpentOnExitCMP + nextStepDuration)) * (1.0 - Math.exp(-omega * nextNextTimeSpentOnEntryCMP));

         Assert.assertEquals(firstExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstExitCMPMultiplier));
         Assert.assertFalse(Double.isNaN(secondExitCMPMultiplier));
         for (int i = 2; i < maxSteps; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultipliers.compute(2, 3, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         firstExitCMPMultiplier = Math.exp(-omega * currentStepDuration) * (1.0 - Math.exp(-omega * upcomingTimeSpentOnEntryCMP));
         secondExitCMPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration)) * (1.0 - Math.exp(-omega * nextNextTimeSpentOnEntryCMP));

         Assert.assertEquals(firstExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondExitCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstExitCMPMultiplier));
         Assert.assertFalse(Double.isNaN(secondExitCMPMultiplier));
         for (int i = 2; i < maxSteps; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepsOneRegisteredOneCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 1;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < stepsRegistered; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());

         boolean isInTransfer = false;
         boolean useTwoCMPs = false;

         entryCMPRecursionMultipliers.compute(2, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double nextNextStepDuration = doubleSupportDurations.get(2).getDoubleValue() + singleSupportDurations.get(2).getDoubleValue();


         double firstEntryCMPMultiplier = Math.exp(-omega * currentStepDuration) * (1.0 - Math.exp(-omega * nextStepDuration));
         double secondEntryCMPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration)) * (1.0 - Math.exp(-omega * nextNextStepDuration));

         Assert.assertEquals(firstEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstEntryCMPMultiplier));
         for (int i = 1; i < maxSteps; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultipliers.compute(2, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(firstEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstEntryCMPMultiplier));
         for (int i = 1; i < maxSteps; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepsTwoRegisteredOneCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 2;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < stepsRegistered; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());

         boolean isInTransfer = false;
         boolean useTwoCMPs = false;

         entryCMPRecursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double nextNextStepDuration = doubleSupportDurations.get(2).getDoubleValue() + singleSupportDurations.get(2).getDoubleValue();


         double firstEntryCMPMultiplier = Math.exp(-omega * currentStepDuration) * (1.0 - Math.exp(-omega * nextStepDuration));
         double secondEntryCMPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration)) * (1.0 - Math.exp(-omega * nextNextStepDuration));

         Assert.assertEquals(firstEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstEntryCMPMultiplier));
         Assert.assertFalse(Double.isNaN(secondEntryCMPMultiplier));
         for (int i = 2; i < maxSteps; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(firstEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstEntryCMPMultiplier));
         Assert.assertFalse(Double.isNaN(secondEntryCMPMultiplier));
         for (int i = 2; i < maxSteps; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepsThreeRegisteredOneCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int stepsRegistered = 3;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < stepsRegistered; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }
         doubleSupportDurations.get(stepsRegistered).set(2.0 * random.nextDouble());

         boolean isInTransfer = false;
         boolean useTwoCMPs = false;

         entryCMPRecursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double nextNextStepDuration = doubleSupportDurations.get(2).getDoubleValue() + singleSupportDurations.get(2).getDoubleValue();


         double firstEntryCMPMultiplier = Math.exp(-omega * currentStepDuration) * (1.0 - Math.exp(-omega * nextStepDuration));
         double secondEntryCMPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration)) * (1.0 - Math.exp(-omega * nextNextStepDuration));

         Assert.assertEquals(firstEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstEntryCMPMultiplier));
         Assert.assertFalse(Double.isNaN(secondEntryCMPMultiplier));
         for (int i = 2; i < maxSteps; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));

         // setup for in transfer
         isInTransfer = true;
         entryCMPRecursionMultipliers.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(firstEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(0), epsilon);
         Assert.assertEquals(secondEntryCMPMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(1), epsilon);
         Assert.assertFalse(Double.isNaN(firstEntryCMPMultiplier));
         Assert.assertFalse(Double.isNaN(secondEntryCMPMultiplier));
         for (int i = 2; i < maxSteps; i++)
            Assert.assertTrue(Double.isNaN(entryCMPRecursionMultipliers.getEntryMultiplier(i)));
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
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, exitCMPRatio, registry);

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

            entryCMPRecursionMultipliers.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
            double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;

            double recursionTime = currentTimeSpentOnExitCMP;
            for (int i = 0; i < j; i++)
            {
               double stepDuration = doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();
               double timeSpentOnEntryCMP = (1.0 - exitRatio) * stepDuration;
               double entryMultiplier = Math.exp(-omega * recursionTime) * ( 1.0 - Math.exp(-omega * timeSpentOnEntryCMP));

               Assert.assertEquals("j = " + j + ", i = " + i, entryMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(i), epsilon);

               recursionTime += stepDuration;
            }

            // setup for in transfer
            isInTransfer = true;
            entryCMPRecursionMultipliers.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            recursionTime = currentStepDuration;
            for (int i = 0; i < j; i++)
            {
               double stepDuration = doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();
               double timeSpentOnEntryCMP = (1.0 - exitRatio) * stepDuration;
               double entryMultiplier = Math.exp(-omega * recursionTime) * ( 1.0 - Math.exp(-omega * timeSpentOnEntryCMP));

               Assert.assertEquals("j = " + j + ", i = " + i, entryMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(i), epsilon);

               recursionTime += stepDuration;
            }
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testNStepTwoCMPCalculationFinalTransfer()
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
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, exitCMPRatio, registry);

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

            entryCMPRecursionMultipliers.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
            double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;

            double recursionTime = currentTimeSpentOnExitCMP;
            for (int i = 0; i < j; i++)
            {
               double stepDuration = doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();
               double timeSpentOnEntryCMP = (1.0 - exitRatio) * stepDuration;
               double entryMultiplier = Math.exp(-omega * recursionTime) * ( 1.0 - Math.exp(-omega * timeSpentOnEntryCMP));

               Assert.assertEquals("j = " + j + ", i = " + i, entryMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(i), epsilon);

               recursionTime += stepDuration;
            }

            // setup for in transfer
            isInTransfer = true;
            entryCMPRecursionMultipliers.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            recursionTime = currentStepDuration;
            for (int i = 0; i < j; i++)
            {
               double stepDuration = doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();
               double timeSpentOnEntryCMP = (1.0 - exitRatio) * stepDuration;
               double entryMultiplier = Math.exp(-omega * recursionTime) * ( 1.0 - Math.exp(-omega * timeSpentOnEntryCMP));

               Assert.assertEquals("j = " + j + ", i = " + i, entryMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(i), epsilon);

               recursionTime += stepDuration;
            }
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
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, exitCMPRatio, registry);

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

            entryCMPRecursionMultipliers.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();

            double recursionTime = currentStepDuration;
            for (int i = 0; i < j; i++)
            {
               double stepDuration = doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();

               double entryMultiplier = Math.exp(-omega * recursionTime) * (1.0 - Math.exp(-omega * stepDuration));

               Assert.assertEquals(entryMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(i), epsilon);

               recursionTime += stepDuration;
            }

            // setup for in transfer
            isInTransfer = true;
            entryCMPRecursionMultipliers.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            recursionTime = currentStepDuration;
            for (int i = 0; i < j; i++)
            {
               double stepDuration = doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();

               double entryMultiplier = Math.exp(-omega * recursionTime) * (1.0 - Math.exp(-omega * stepDuration));

               Assert.assertEquals(entryMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(i), epsilon);

               recursionTime += stepDuration;
            }
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testNStepOneCMPCalculationInTransfer()
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
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      EntryCMPRecursionMultipliers entryCMPRecursionMultipliers = new EntryCMPRecursionMultipliers("", maxSteps, exitCMPRatio, registry);

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

            entryCMPRecursionMultipliers.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();

            double recursionTime = currentStepDuration;
            for (int i = 0; i < j; i++)
            {
               double stepDuration = doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();

               double entryMultiplier = Math.exp(-omega * recursionTime) * (1.0 - Math.exp(-omega * stepDuration));

               Assert.assertEquals(entryMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(i), epsilon);

               recursionTime += stepDuration;
            }

            // setup for in transfer
            isInTransfer = true;
            entryCMPRecursionMultipliers.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            recursionTime = currentStepDuration;
            for (int i = 0; i < j; i++)
            {
               double stepDuration = doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();

               double entryMultiplier = Math.exp(-omega * recursionTime) * (1.0 - Math.exp(-omega * stepDuration));

               Assert.assertEquals(entryMultiplier, entryCMPRecursionMultipliers.getEntryMultiplier(i), epsilon);

               recursionTime += stepDuration;
            }
         }
      }
   }
}

