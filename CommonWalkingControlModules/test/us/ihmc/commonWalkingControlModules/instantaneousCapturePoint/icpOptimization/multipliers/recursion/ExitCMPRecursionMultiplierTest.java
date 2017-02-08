package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import java.util.ArrayList;
import java.util.Random;

public class ExitCMPRecursionMultiplierTest
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

      ExitCMPRecursionMultiplier exitCMPRecursionMultiplier = new ExitCMPRecursionMultiplier("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }


         // setup for in swing
         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         exitCMPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double upcomingStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;
         double upcomingTimeSpentOnEntryCMP = (1 - exitRatio) * upcomingStepDuration;
         double upcomingTimeSpentOnExitCMP = exitRatio * upcomingStepDuration;

         double exitCMPMultiplier = Math.exp(-omega * (currentTimeSpentOnExitCMP + upcomingTimeSpentOnEntryCMP)) * (1.0 - Math.exp(-omega * upcomingTimeSpentOnExitCMP));
         Assert.assertEquals(exitCMPMultiplier, exitCMPRecursionMultiplier.getExitMultiplier(0), epsilon);

         // setup for in transfer
         isInTransfer = true;
         exitCMPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         exitCMPMultiplier = Math.exp(-omega * (currentStepDuration + upcomingTimeSpentOnEntryCMP)) * (1.0 - Math.exp(-omega * upcomingTimeSpentOnExitCMP));
         Assert.assertEquals(exitCMPMultiplier, exitCMPRecursionMultiplier.getExitMultiplier(0), epsilon);
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

      ExitCMPRecursionMultiplier exitCMPRecursionMultiplier = new ExitCMPRecursionMultiplier("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }


         // setup for in swing
         boolean isInTransfer = false;
         boolean useTwoCMPs = false;

         exitCMPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPRecursionMultiplier.getExitMultiplier(0), epsilon);

         // setup for in transfer
         isInTransfer = true;
         exitCMPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPRecursionMultiplier.getExitMultiplier(0), epsilon);
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

      ExitCMPRecursionMultiplier exitCMPRecursionMultiplier = new ExitCMPRecursionMultiplier("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }


         // setup for in swing
         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         exitCMPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double nextStepDuration = doubleSupportDurations.get(1).getDoubleValue() + singleSupportDurations.get(1).getDoubleValue();
         double nextNextStepDuration = doubleSupportDurations.get(2).getDoubleValue() + singleSupportDurations.get(2).getDoubleValue();

         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;
         double nextTimeSpentOnEntryCMP = (1 - exitRatio) * nextStepDuration;
         double nextNextTimeSpentOnEntryCMP = (1 - exitRatio) * nextNextStepDuration;
         double nextTimeSpentOnExitCMP = exitRatio * nextStepDuration;
         double nextNextTimeSpentOnExitCMP = exitRatio * nextNextStepDuration;

         double firstExitCMPMultiplier = Math.exp(-omega * (currentTimeSpentOnExitCMP + nextTimeSpentOnEntryCMP)) * (1.0 - Math.exp(-omega * nextTimeSpentOnExitCMP));
         double secondExitCMPMultiplier = Math.exp(-omega * (currentTimeSpentOnExitCMP + nextStepDuration + nextNextTimeSpentOnEntryCMP)) * (1.0 - Math.exp(-omega * nextNextTimeSpentOnExitCMP));

         Assert.assertEquals(firstExitCMPMultiplier, exitCMPRecursionMultiplier.getExitMultiplier(0), epsilon);
         Assert.assertEquals(secondExitCMPMultiplier, exitCMPRecursionMultiplier.getExitMultiplier(1), epsilon);

         // setup for in transfer
         isInTransfer = true;
         exitCMPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         firstExitCMPMultiplier = Math.exp(-omega * (currentStepDuration + nextTimeSpentOnEntryCMP)) * (1.0 - Math.exp(-omega * nextTimeSpentOnExitCMP));
         secondExitCMPMultiplier = Math.exp(-omega * (currentStepDuration + nextStepDuration + nextNextTimeSpentOnEntryCMP)) * (1.0 - Math.exp(-omega * nextNextTimeSpentOnExitCMP));

         Assert.assertEquals(firstExitCMPMultiplier, exitCMPRecursionMultiplier.getExitMultiplier(0), epsilon);
         Assert.assertEquals(secondExitCMPMultiplier, exitCMPRecursionMultiplier.getExitMultiplier(1), epsilon);
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

      ExitCMPRecursionMultiplier exitCMPRecursionMultiplier = new ExitCMPRecursionMultiplier("", maxSteps, exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }


         // setup for in swing
         boolean isInTransfer = false;
         boolean useTwoCMPs = false;

         exitCMPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPRecursionMultiplier.getExitMultiplier(0), epsilon);
         Assert.assertEquals(0.0, exitCMPRecursionMultiplier.getExitMultiplier(1), epsilon);

         // setup for in transfer
         isInTransfer = true;
         exitCMPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPRecursionMultiplier.getExitMultiplier(0), epsilon);
         Assert.assertEquals(0.0, exitCMPRecursionMultiplier.getExitMultiplier(1), epsilon);
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

      ExitCMPRecursionMultiplier exitCMPRecursionMultiplier = new ExitCMPRecursionMultiplier("", maxSteps, exitCMPRatio, registry);

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

            // setup for in swing
            boolean isInTransfer = false;
            boolean useTwoCMPs = true;

            exitCMPRecursionMultiplier.compute(j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
            double recursionTime = exitRatio * currentStepDuration;

            for (int i = 0; i < j; i++)
            {
               double stepDuration = doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();
               double timeSpentOnEntryCMP = (1.0 - exitRatio) * stepDuration;
               double timeSpentOnExitCMP = exitRatio * stepDuration;

               double exitMultiplier = Math.exp(-omega * (timeSpentOnEntryCMP + recursionTime)) * ( 1.0 - Math.exp(-omega * timeSpentOnExitCMP));

               Assert.assertEquals(exitMultiplier, exitCMPRecursionMultiplier.getExitMultiplier(i), epsilon);

               recursionTime += stepDuration;
            }

            // setup for in transfer
            isInTransfer = true;
            exitCMPRecursionMultiplier.compute(j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            recursionTime = currentStepDuration;
            for (int i = 0; i < j; i++)
            {
               double stepDuration = doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();
               double timeSpentOnEntryCMP = (1.0 - exitRatio) * stepDuration;
               double timeSpentOnExitCMP = exitRatio * stepDuration;

               double exitMultiplier = Math.exp(-omega * (timeSpentOnEntryCMP + recursionTime)) * ( 1.0 - Math.exp(-omega * timeSpentOnExitCMP));

               Assert.assertEquals(exitMultiplier, exitCMPRecursionMultiplier.getExitMultiplier(i), epsilon);

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
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      ExitCMPRecursionMultiplier exitCMPRecursionMultiplier = new ExitCMPRecursionMultiplier("", maxSteps, exitCMPRatio, registry);

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

            // setup for in swing
            boolean isInTransfer = false;
            boolean useTwoCMPs = false;

            exitCMPRecursionMultiplier.compute(j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            for (int i = 0; i < j ; i++)
            {
               Assert.assertEquals(0.0, exitCMPRecursionMultiplier.getExitMultiplier(i), epsilon);
            }

            // setup for in transfer
            isInTransfer = true;
            exitCMPRecursionMultiplier.compute(j, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

            for (int i = 0; i < j ; i++)
            {
               Assert.assertEquals(0.0, exitCMPRecursionMultiplier.getExitMultiplier(i), epsilon);
            }
         }
      }
   }
}
