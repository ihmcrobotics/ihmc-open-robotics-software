package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class FinalICPRecursionMultiplierTest
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

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

         finalICPRecursionMultiplier.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double upcomingStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;

         double finalICPMultiplier = Math.exp(-omega * (currentTimeSpentOnExitCMP + upcomingStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));

         // setup for in transfer
         isInTransfer = true;
         finalICPRecursionMultiplier.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         finalICPMultiplier = Math.exp(-omega * (currentStepDuration + upcomingStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneStepTwoCMPCalculationFinalStep()
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
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

         finalICPRecursionMultiplier.compute(1, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double upcomingStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;

         double finalICPMultiplier = Math.exp(-omega * (currentTimeSpentOnExitCMP + upcomingStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));

         // setup for in transfer
         isInTransfer = true;
         finalICPRecursionMultiplier.compute(1, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         finalICPMultiplier = Math.exp(-omega * (currentStepDuration + upcomingStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
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

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

         finalICPRecursionMultiplier.compute(1, 2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double upcomingStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * (currentStepDuration + upcomingStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));

         // setup for in transfer
         isInTransfer = true;
         finalICPRecursionMultiplier.compute(1, 2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneStepOneCMPCalculationFinalStep()
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

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

         finalICPRecursionMultiplier.compute(1, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double upcomingStepDuration = doubleSupportDurations.get(1).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * (currentStepDuration + upcomingStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));

         // setup for in transfer
         isInTransfer = true;
         finalICPRecursionMultiplier.compute(1, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepOneStepRegisteredTwoCMPCalculation()
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

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

         finalICPRecursionMultiplier.compute(2, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;

         double finalICPMultiplier = Math.exp(-omega * (currentTimeSpentOnExitCMP + nextStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));

         // setup for in transfer
         isInTransfer = true;
         finalICPRecursionMultiplier.compute(2, 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         finalICPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepTwoStepRegisteredTwoCMPCalculation()
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

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

         finalICPRecursionMultiplier.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double nextNextStepDuration = doubleSupportDurations.get(2).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;

         double finalICPMultiplier = Math.exp(-omega * (currentTimeSpentOnExitCMP + nextStepDuration + nextNextStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));

         // setup for in transfer
         isInTransfer = true;
         finalICPRecursionMultiplier.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         finalICPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration + nextNextStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepThreeStepRegisteredTwoCMPCalculation()
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

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

         finalICPRecursionMultiplier.compute(2, 3, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double nextNextStepDuration = doubleSupportDurations.get(2).getDoubleValue() + singleSupportDurations.get(2).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;

         double finalICPMultiplier = Math.exp(-omega * (currentTimeSpentOnExitCMP + nextStepDuration + nextNextStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));

         // setup for in transfer
         isInTransfer = true;
         finalICPRecursionMultiplier.compute(2, 3, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         finalICPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration + nextNextStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepOneStepRegisteredOneCMPCalculation()
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

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

         finalICPRecursionMultiplier.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));

         // setup for in transfer
         isInTransfer = true;
         finalICPRecursionMultiplier.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepTwoStepRegisteredOneCMPCalculation()
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

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

         finalICPRecursionMultiplier.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double nextNextStepDuration = doubleSupportDurations.get(2).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration + nextNextStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));

         // setup for in transfer
         isInTransfer = true;
         finalICPRecursionMultiplier.compute(2, stepsRegistered, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConsiderTwoStepThreeStepRegisteredOneCMPCalculation()
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

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

         finalICPRecursionMultiplier.compute(2, 3, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double nextNextStepDuration = doubleSupportDurations.get(2).getDoubleValue() + singleSupportDurations.get(2).getDoubleValue();

         double finalICPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration + nextNextStepDuration));
         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));

         // setup for in transfer
         isInTransfer = true;
         finalICPRecursionMultiplier.compute(2, 3, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
         Assert.assertFalse(Double.isNaN(finalICPMultiplier));
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

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

            finalICPRecursionMultiplier.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
            double recursionTime = exitRatio * currentStepDuration;

            for (int i = 0; i < j; i++)
               recursionTime += doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();

            double finalICPMultiplier = Math.exp(-omega * recursionTime);
            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
            Assert.assertFalse(Double.isNaN(finalICPMultiplier));

            // setup for in transfer
            isInTransfer = true;
            finalICPRecursionMultiplier.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            recursionTime = currentStepDuration;
            for (int i = 0; i < j; i++)
               recursionTime += doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();

            finalICPMultiplier = Math.exp(-omega * recursionTime);
            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
            Assert.assertFalse(Double.isNaN(finalICPMultiplier));
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

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

            finalICPRecursionMultiplier.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
            double recursionTime = exitRatio * currentStepDuration;

            for (int i = 0; i < j; i++)
            {
               boolean isLast = i + 1 == j;
               if (isLast)
                  recursionTime += doubleSupportDurations.get(i + 1).getDoubleValue();
               else
                  recursionTime += doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();
            }


            double finalICPMultiplier = Math.exp(-omega * recursionTime);
            Assert.assertFalse(Double.isNaN(finalICPMultiplier));
            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);

            // setup for in transfer
            isInTransfer = true;
            finalICPRecursionMultiplier.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            recursionTime = currentStepDuration;
            for (int i = 0; i < j; i++)
            {
               boolean isLast = i + 1 == j;
               if (isLast)
                  recursionTime += doubleSupportDurations.get(i + 1).getDoubleValue();
               else
                  recursionTime += doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();
            }

            finalICPMultiplier = Math.exp(-omega * recursionTime);
            Assert.assertFalse(Double.isNaN(finalICPMultiplier));
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
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

            finalICPRecursionMultiplier.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
            double recursionTime = currentStepDuration;
            for (int i = 0; i < j; i++)
            {
               recursionTime += doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();
            }

            double finalICPMultiplier = Math.exp(-omega * recursionTime);
            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
            Assert.assertFalse(Double.isNaN(finalICPMultiplier));

            // setup for in transfer
            isInTransfer = true;
            finalICPRecursionMultiplier.compute(j, j + 1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
            Assert.assertFalse(Double.isNaN(finalICPMultiplier));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testNStepOneCMPCalculationFinalTransfer()
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

      FinalICPRecursionMultiplier finalICPRecursionMultiplier = new FinalICPRecursionMultiplier("", exitCMPRatio, registry);

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

            finalICPRecursionMultiplier.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
            double recursionTime = currentStepDuration;
            for (int i = 0; i < j; i++)
            {
               boolean isLast = i + 1 == j;
               if (isLast)
                  recursionTime += doubleSupportDurations.get(i + 1).getDoubleValue();
               else
                  recursionTime += doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();
            }

            double finalICPMultiplier = Math.exp(-omega * recursionTime);
            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
            Assert.assertFalse(Double.isNaN(finalICPMultiplier));

            // setup for in transfer
            isInTransfer = true;
            finalICPRecursionMultiplier.compute(j, j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            Assert.assertEquals(finalICPMultiplier, finalICPRecursionMultiplier.getDoubleValue(), epsilon);
            Assert.assertFalse(Double.isNaN(finalICPMultiplier));
         }
      }
   }
}
