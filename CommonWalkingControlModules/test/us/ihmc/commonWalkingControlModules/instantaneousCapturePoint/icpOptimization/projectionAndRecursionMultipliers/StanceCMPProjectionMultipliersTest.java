package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.StanceCMPProjectionMultipliers;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import java.util.ArrayList;
import java.util.Random;

public class StanceCMPProjectionMultipliersTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      Random random = new Random();
      int iters = 100;

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable("exitCMPDurationInPercentOfStepTime", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("currentDoubleSupportDuration", registry));
      doubleSupportDurations.add(new DoubleYoVariable("upcomingDoubleSupportDuration", registry));
      singleSupportDurations.add(new DoubleYoVariable("singleSupportduration", registry));

      StanceCMPProjectionMultipliers stanceCMPProjectionMultipliers = new StanceCMPProjectionMultipliers("", doubleSupportSplitRatio,
            upcomingDoubleSupportSplitRatio, exitCMPDurationInPercentOfStepTime, registry);

      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfStepTime.set(exitRatio);

         double currentDoubleSupportDuration = 1.0 * random.nextDouble();
         double upcomingDoubleSupportDuration = 1.0 * random.nextDouble();
         double singleSupportDuration = 2.0 * random.nextDouble();

         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         doubleSupportDurations.get(1).set(upcomingDoubleSupportDuration);
         singleSupportDurations.get(0).set(singleSupportDuration);

         double upcomingInitialDoubleSupportDuration = splitRatio * upcomingDoubleSupportDuration;
         double endOfDoubleSupport = (1.0 - splitRatio) * currentDoubleSupportDuration;

         double timeOnEntry = (1.0 - exitRatio) * (currentDoubleSupportDuration + singleSupportDuration);
         double timeOnExit = exitRatio * (currentDoubleSupportDuration + singleSupportDuration);

         boolean useTwoCMPs = true;
         boolean isInTransfer= false;

         double exitMultiplier = (1.0 - Math.exp(-omega0 * upcomingInitialDoubleSupportDuration));
         double entryMultiplier = 0.0;

         stanceCMPProjectionMultipliers.compute(upcomingDoubleSupportDuration, currentDoubleSupportDuration, singleSupportDuration, useTwoCMPs, isInTransfer, omega0, 1);
         Assert.assertEquals("", exitMultiplier, stanceCMPProjectionMultipliers.getExitMultiplier(), epsilon);
         Assert.assertEquals("", entryMultiplier, stanceCMPProjectionMultipliers.getEntryMultiplier(), epsilon);

         stanceCMPProjectionMultipliers.compute(doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0, 1);
         Assert.assertEquals("", exitMultiplier, stanceCMPProjectionMultipliers.getExitMultiplier(), epsilon);
         Assert.assertEquals("", entryMultiplier, stanceCMPProjectionMultipliers.getEntryMultiplier(), epsilon);

         isInTransfer = true;

         exitMultiplier = Math.exp(-omega0 * (timeOnEntry - endOfDoubleSupport)) * (1.0 - Math.exp(-omega0 * timeOnExit));
         entryMultiplier = 1.0 - Math.exp(-omega0 * (timeOnEntry - endOfDoubleSupport));

         stanceCMPProjectionMultipliers.compute(upcomingDoubleSupportDuration, currentDoubleSupportDuration, singleSupportDuration, useTwoCMPs, isInTransfer, omega0, 1);

         Assert.assertEquals("", exitMultiplier, stanceCMPProjectionMultipliers.getExitMultiplier(), epsilon);
         Assert.assertEquals("", entryMultiplier, stanceCMPProjectionMultipliers.getEntryMultiplier(), epsilon);

         stanceCMPProjectionMultipliers.compute(doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0, 1);
         Assert.assertEquals("", exitMultiplier, stanceCMPProjectionMultipliers.getExitMultiplier(), epsilon);
         Assert.assertEquals("", entryMultiplier, stanceCMPProjectionMultipliers.getEntryMultiplier(), epsilon);

         useTwoCMPs = false;
         isInTransfer = false;

         exitMultiplier = (1.0 - Math.exp(-omega0 * upcomingInitialDoubleSupportDuration));
         entryMultiplier = 0.0;

         stanceCMPProjectionMultipliers.compute(upcomingDoubleSupportDuration, currentDoubleSupportDuration, singleSupportDuration, useTwoCMPs, isInTransfer, omega0, 1);

         Assert.assertEquals("", exitMultiplier, stanceCMPProjectionMultipliers.getExitMultiplier(), epsilon);
         Assert.assertEquals("", entryMultiplier, stanceCMPProjectionMultipliers.getEntryMultiplier(), epsilon);

         stanceCMPProjectionMultipliers.compute(doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0, 1);
         Assert.assertEquals("", exitMultiplier, stanceCMPProjectionMultipliers.getExitMultiplier(), epsilon);
         Assert.assertEquals("", entryMultiplier, stanceCMPProjectionMultipliers.getEntryMultiplier(), epsilon);

         useTwoCMPs = false;
         isInTransfer = true;

         exitMultiplier = (1.0 - Math.exp(-omega0 * (upcomingInitialDoubleSupportDuration + singleSupportDuration)));
         entryMultiplier = 0.0;

         stanceCMPProjectionMultipliers.compute(upcomingDoubleSupportDuration, currentDoubleSupportDuration, singleSupportDuration, useTwoCMPs, isInTransfer, omega0, 1);

         Assert.assertEquals("", exitMultiplier, stanceCMPProjectionMultipliers.getExitMultiplier(), epsilon);
         Assert.assertEquals("", entryMultiplier, stanceCMPProjectionMultipliers.getEntryMultiplier(), epsilon);

         stanceCMPProjectionMultipliers.compute(doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega0, 1);
         Assert.assertEquals("", exitMultiplier, stanceCMPProjectionMultipliers.getExitMultiplier(), epsilon);
         Assert.assertEquals("", entryMultiplier, stanceCMPProjectionMultipliers.getEntryMultiplier(), epsilon);

         stanceCMPProjectionMultipliers.reset();
         ;
         Assert.assertEquals("", 0.0, stanceCMPProjectionMultipliers.getExitMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, stanceCMPProjectionMultipliers.getEntryMultiplier(), epsilon);
      }
   }
}
