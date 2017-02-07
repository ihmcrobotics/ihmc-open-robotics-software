package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import java.util.ArrayList;
import java.util.Random;

public class NewInitialICPCurrentMultiplierTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationProjectForwardTwoCMPFirstSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable defaultSplitRatio = new DoubleYoVariable("defaultSplitRatio", registry);
      DoubleYoVariable upcomingSplitRatio = new DoubleYoVariable("upcomingSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
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

      boolean projectCMPForward = true;
      NewInitialICPCurrentMultiplier initialICPCurrentMultiplier = new NewInitialICPCurrentMultiplier(upcomingSplitRatio, defaultSplitRatio, exitCMPRatio, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime, projectCMPForward, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);
         double defaultSplit = 0.7 * random.nextDouble();
         defaultSplitRatio.set(defaultSplit);
         double upcomingSplit = 0.7 * random.nextDouble();
         upcomingSplitRatio.set(upcomingSplit);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * startOfSpline;
         double timeRemaining = singleSupportDuration - timeInCurrentState;

         initialICPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega);

         double projection = Math.exp(omega * timeInCurrentState);
         Assert.assertEquals(1.0 - projection, initialICPCurrentMultiplier.getPositionMultiplier(), epsilon);

         Assert.assertEquals(- omega * projection, initialICPCurrentMultiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationRecurseBackwardTwoCMPFirstSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable defaultSplitRatio = new DoubleYoVariable("defaultSplitRatio", registry);
      DoubleYoVariable upcomingSplitRatio = new DoubleYoVariable("upcomingSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
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

      boolean projectCMPForward = false;
      NewInitialICPCurrentMultiplier initialICPCurrentMultiplier = new NewInitialICPCurrentMultiplier(upcomingSplitRatio, defaultSplitRatio, exitCMPRatio, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime, projectCMPForward, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);
         double defaultSplit = 0.7 * random.nextDouble();
         defaultSplitRatio.set(defaultSplit);
         double upcomingSplit = 0.7 * random.nextDouble();
         upcomingSplitRatio.set(upcomingSplit);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * startOfSpline;
         double timeRemaining = singleSupportDuration - timeInCurrentState;

         initialICPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega);

         double doubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();
         double timeSpentOnEntryCMP = (1.0 - exitRatio) * (singleSupportDuration + doubleSupportDuration);
         double endOfDoubleSupportDuration = (1.0 - defaultSplit) * doubleSupportDuration;
         double initialSingleSupportTime = timeSpentOnEntryCMP - endOfDoubleSupportDuration;

         double projectionTime = timeInCurrentState - initialSingleSupportTime;
         double projection = Math.exp(omega * projectionTime);

         Assert.assertEquals("iter = " + iter, 1.0 - projection, initialICPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("iter = " + iter, -omega * projection, initialICPCurrentMultiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationProjectForwardTwoCMPSecondSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable defaultSplitRatio = new DoubleYoVariable("defaultSplitRatio", registry);
      DoubleYoVariable upcomingSplitRatio = new DoubleYoVariable("upcomingSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
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

      boolean projectCMPForward = true;
      NewInitialICPCurrentMultiplier initialICPCurrentMultiplier = new NewInitialICPCurrentMultiplier(upcomingSplitRatio, defaultSplitRatio, exitCMPRatio, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime, projectCMPForward, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);
         double defaultSplit = 0.7 * random.nextDouble();
         defaultSplitRatio.set(defaultSplit);
         double upcomingSplit = 0.7 * random.nextDouble();
         upcomingSplitRatio.set(upcomingSplit);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * (endOfSpline - startOfSpline) + startOfSpline;
         double timeRemaining = singleSupportDuration - timeInCurrentState;

         initialICPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega);

         double projection = Math.exp(omega * timeInCurrentState);
         Assert.assertEquals(1.0 - projection, initialICPCurrentMultiplier.getPositionMultiplier(), epsilon);

         Assert.assertEquals(- omega * projection, initialICPCurrentMultiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationRecurseBackwardTwoCMPSecondSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable defaultSplitRatio = new DoubleYoVariable("defaultSplitRatio", registry);
      DoubleYoVariable upcomingSplitRatio = new DoubleYoVariable("upcomingSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
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

      boolean projectCMPForward = false;
      NewInitialICPCurrentMultiplier initialICPCurrentMultiplier = new NewInitialICPCurrentMultiplier(upcomingSplitRatio, defaultSplitRatio, exitCMPRatio, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime, projectCMPForward, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);
         double defaultSplit = 0.7 * random.nextDouble();
         defaultSplitRatio.set(defaultSplit);
         double upcomingSplit = 0.7 * random.nextDouble();
         upcomingSplitRatio.set(upcomingSplit);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * (endOfSpline - startOfSpline) + startOfSpline;
         double timeRemaining = singleSupportDuration - timeInCurrentState;

         initialICPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega);

         double projection = Math.exp(omega * timeInCurrentState);
         Assert.assertEquals(1.0 - projection, initialICPCurrentMultiplier.getPositionMultiplier(), epsilon);

         Assert.assertEquals(- omega * projection, initialICPCurrentMultiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationProjectForwardTwoCMPThirdSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable defaultSplitRatio = new DoubleYoVariable("defaultSplitRatio", registry);
      DoubleYoVariable upcomingSplitRatio = new DoubleYoVariable("upcomingSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
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

      boolean projectCMPForward = true;
      NewInitialICPCurrentMultiplier initialICPCurrentMultiplier = new NewInitialICPCurrentMultiplier(upcomingSplitRatio, defaultSplitRatio, exitCMPRatio, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime, projectCMPForward, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);
         double defaultSplit = 0.7 * random.nextDouble();
         defaultSplitRatio.set(defaultSplit);
         double upcomingSplit = 0.7 * random.nextDouble();
         upcomingSplitRatio.set(upcomingSplit);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * (singleSupportDuration - endOfSpline) + endOfSpline;
         double timeRemaining = singleSupportDuration - timeInCurrentState;

         initialICPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega);

         double projection = Math.exp(omega * timeInCurrentState);
         Assert.assertEquals(1.0 - projection, initialICPCurrentMultiplier.getPositionMultiplier(), epsilon);

         Assert.assertEquals(- omega * projection, initialICPCurrentMultiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationRecurseBackwardTwoCMPThirdSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable defaultSplitRatio = new DoubleYoVariable("defaultSplitRatio", registry);
      DoubleYoVariable upcomingSplitRatio = new DoubleYoVariable("upcomingSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
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

      boolean projectCMPForward = false;
      NewInitialICPCurrentMultiplier initialICPCurrentMultiplier = new NewInitialICPCurrentMultiplier(upcomingSplitRatio, defaultSplitRatio, exitCMPRatio, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime, projectCMPForward, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);
         double defaultSplit = 0.7 * random.nextDouble();
         defaultSplitRatio.set(defaultSplit);
         double upcomingSplit = 0.7 * random.nextDouble();
         upcomingSplitRatio.set(upcomingSplit);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * (singleSupportDuration - endOfSpline) + endOfSpline;
         double timeRemaining = singleSupportDuration - timeInCurrentState;

         initialICPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega);

         double projection = Math.exp(omega * timeInCurrentState);
         Assert.assertEquals(1.0 - projection, initialICPCurrentMultiplier.getPositionMultiplier(), epsilon);

         Assert.assertEquals(- omega * projection, initialICPCurrentMultiplier.getVelocityMultiplier(), epsilon);
      }
   }
}
