package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers;

import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import org.junit.Assert;

import java.util.ArrayList;
import java.util.Random;

public class RemainingStanceCMPProjectionMultipliersTest
{
   private static final double epsilon = 0.0005;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
      DoubleYoVariable exitRatio = new DoubleYoVariable("exitRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      ExitCMPProjectionMultiplier exitMultiplier = new ExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitRatio, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);
      EntryCMPProjectionMultiplier entryMultiplier = new EntryCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, exitRatio, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);
      PreviousExitCMPProjectionMultiplier previosuMultiplier = new PreviousExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio);

      RemainingStanceCMPProjectionMultipliers remainingMultipliers = new RemainingStanceCMPProjectionMultipliers(defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitRatio, startOfSplineTime, endOfSplineTime, totalTrajectoryTime, registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("currentDoubleSupportDuration", registry));
      doubleSupportDurations.add(new DoubleYoVariable("upcomingDoubleSupportDuration", registry));
      singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration", registry));

      double omega0 = 3.0;
      Random random = new Random();

      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         boolean useTwoCMPs = true;
         boolean isInTransfer = true;
         boolean useInitialICP = true;

         double currentDoubleSupportDuration = 3.0 * random.nextDouble();
         double upcomingDoubleSupportDuration = 3.0 * random.nextDouble();
         double singleSupportDuration = 3.0 * random.nextDouble();

         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         doubleSupportDurations.get(1).set(upcomingDoubleSupportDuration);
         singleSupportDurations.get(0).set(singleSupportDuration);

         double timeRemaining = random.nextDouble() * currentDoubleSupportDuration;

         remainingMultipliers.compute(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         exitMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         entryMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         previosuMultiplier.compute(doubleSupportDurations, timeRemaining, isInTransfer, omega0, useInitialICP);

         Assert.assertEquals(exitMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingExitMultiplier(), epsilon);
         Assert.assertEquals(exitMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingExitVelocityMultiplier(), epsilon);

         Assert.assertEquals(entryMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingEntryMultiplier(), epsilon);
         Assert.assertEquals(entryMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingEntryVelocityMultiplier(), epsilon);

         Assert.assertEquals(previosuMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingPreviousExitMultiplier(), epsilon);
         Assert.assertEquals(previosuMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingPreviousExitVelocityMultiplier(), epsilon);

         useInitialICP = false;

         remainingMultipliers.compute(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         exitMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         entryMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         previosuMultiplier.compute(doubleSupportDurations, timeRemaining, isInTransfer, omega0, useInitialICP);

         Assert.assertEquals(exitMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingExitMultiplier(), epsilon);
         Assert.assertEquals(exitMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingExitVelocityMultiplier(), epsilon);

         Assert.assertEquals(entryMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingEntryMultiplier(), epsilon);
         Assert.assertEquals(entryMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingEntryVelocityMultiplier(), epsilon);

         Assert.assertEquals(previosuMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingPreviousExitMultiplier(), epsilon);
         Assert.assertEquals(previosuMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingPreviousExitVelocityMultiplier(), epsilon);

         useTwoCMPs = false;
         useInitialICP = true;

         remainingMultipliers.compute(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         exitMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         entryMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         previosuMultiplier.compute(doubleSupportDurations, timeRemaining, isInTransfer, omega0, useInitialICP);

         Assert.assertEquals(exitMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingExitMultiplier(), epsilon);
         Assert.assertEquals(exitMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingExitVelocityMultiplier(), epsilon);

         Assert.assertEquals(entryMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingEntryMultiplier(), epsilon);
         Assert.assertEquals(entryMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingEntryVelocityMultiplier(), epsilon);

         Assert.assertEquals(previosuMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingPreviousExitMultiplier(), epsilon);
         Assert.assertEquals(previosuMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingPreviousExitVelocityMultiplier(), epsilon);

         useInitialICP = false;

         remainingMultipliers.compute(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         exitMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         entryMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         previosuMultiplier.compute(doubleSupportDurations, timeRemaining, isInTransfer, omega0, useInitialICP);

         Assert.assertEquals(exitMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingExitMultiplier(), epsilon);
         Assert.assertEquals(exitMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingExitVelocityMultiplier(), epsilon);

         Assert.assertEquals(entryMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingEntryMultiplier(), epsilon);
         Assert.assertEquals(entryMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingEntryVelocityMultiplier(), epsilon);

         Assert.assertEquals(previosuMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingPreviousExitMultiplier(), epsilon);
         Assert.assertEquals(previosuMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingPreviousExitVelocityMultiplier(), epsilon);


         timeRemaining = random.nextDouble() * singleSupportDuration;

         isInTransfer = false;
         useInitialICP = true;
         useTwoCMPs = true;

         remainingMultipliers.compute(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         exitMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         entryMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         previosuMultiplier.compute(doubleSupportDurations, timeRemaining, isInTransfer, omega0, useInitialICP);

         Assert.assertEquals(exitMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingExitMultiplier(), epsilon);
         Assert.assertEquals(exitMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingExitVelocityMultiplier(), epsilon);

         Assert.assertEquals(entryMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingEntryMultiplier(), epsilon);
         Assert.assertEquals(entryMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingEntryVelocityMultiplier(), epsilon);

         Assert.assertEquals(previosuMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingPreviousExitMultiplier(), epsilon);
         Assert.assertEquals(previosuMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingPreviousExitVelocityMultiplier(), epsilon);

         useInitialICP = false;

         remainingMultipliers.compute(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         exitMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         entryMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         previosuMultiplier.compute(doubleSupportDurations, timeRemaining, isInTransfer, omega0, useInitialICP);

         Assert.assertEquals(exitMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingExitMultiplier(), epsilon);
         Assert.assertEquals(exitMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingExitVelocityMultiplier(), epsilon);

         Assert.assertEquals(entryMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingEntryMultiplier(), epsilon);
         Assert.assertEquals(entryMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingEntryVelocityMultiplier(), epsilon);

         Assert.assertEquals(previosuMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingPreviousExitMultiplier(), epsilon);
         Assert.assertEquals(previosuMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingPreviousExitVelocityMultiplier(), epsilon);

         useTwoCMPs = false;
         useInitialICP = true;

         remainingMultipliers.compute(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         exitMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         entryMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         previosuMultiplier.compute(doubleSupportDurations, timeRemaining, isInTransfer, omega0, useInitialICP);

         Assert.assertEquals(exitMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingExitMultiplier(), epsilon);
         Assert.assertEquals(exitMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingExitVelocityMultiplier(), epsilon);

         Assert.assertEquals(entryMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingEntryMultiplier(), epsilon);
         Assert.assertEquals(entryMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingEntryVelocityMultiplier(), epsilon);

         Assert.assertEquals(previosuMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingPreviousExitMultiplier(), epsilon);
         Assert.assertEquals(previosuMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingPreviousExitVelocityMultiplier(), epsilon);

         useInitialICP = false;

         remainingMultipliers.compute(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         exitMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         entryMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, useTwoCMPs, isInTransfer, omega0, useInitialICP);
         previosuMultiplier.compute(doubleSupportDurations, timeRemaining, isInTransfer, omega0, useInitialICP);

         Assert.assertEquals(exitMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingExitMultiplier(), epsilon);
         Assert.assertEquals(exitMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingExitVelocityMultiplier(), epsilon);

         Assert.assertEquals(entryMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingEntryMultiplier(), epsilon);
         Assert.assertEquals(entryMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingEntryVelocityMultiplier(), epsilon);

         Assert.assertEquals(previosuMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingPreviousExitMultiplier(), epsilon);
         Assert.assertEquals(previosuMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingPreviousExitVelocityMultiplier(), epsilon);

         exitMultiplier.reset();
         entryMultiplier.reset();
         previosuMultiplier.reset();
         remainingMultipliers.reset();

         Assert.assertEquals(exitMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingExitMultiplier(), epsilon);
         Assert.assertEquals(exitMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingExitVelocityMultiplier(), epsilon);
         Assert.assertEquals(entryMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingEntryMultiplier(), epsilon);
         Assert.assertEquals(entryMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingEntryVelocityMultiplier(), epsilon);
         Assert.assertEquals(previosuMultiplier.getPositionMultiplier(), remainingMultipliers.getRemainingPreviousExitMultiplier(), epsilon);
         Assert.assertEquals(previosuMultiplier.getVelocityMultiplier(), remainingMultipliers.getRemainingPreviousExitVelocityMultiplier(), epsilon);

         Assert.assertEquals(0.0, remainingMultipliers.getRemainingExitMultiplier(), epsilon);
         Assert.assertEquals(0.0, remainingMultipliers.getRemainingExitVelocityMultiplier(), epsilon);
         Assert.assertEquals(0.0, remainingMultipliers.getRemainingEntryMultiplier(), epsilon);
         Assert.assertEquals(0.0, remainingMultipliers.getRemainingEntryVelocityMultiplier(), epsilon);
         Assert.assertEquals(0.0, remainingMultipliers.getRemainingPreviousExitMultiplier(), epsilon);
         Assert.assertEquals(0.0, remainingMultipliers.getRemainingPreviousExitVelocityMultiplier(), epsilon);
      }
   }
}
