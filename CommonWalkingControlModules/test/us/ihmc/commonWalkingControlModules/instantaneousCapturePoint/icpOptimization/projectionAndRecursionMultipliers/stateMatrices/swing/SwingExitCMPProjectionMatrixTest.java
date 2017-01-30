package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing.SwingExitCMPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.MutationTestingTools;

import java.util.ArrayList;
import java.util.Random;

public class SwingExitCMPProjectionMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      SwingExitCMPProjectionMatrix swingExitCMPProjectionMatrix = new SwingExitCMPProjectionMatrix(defaultDoubleSupportSplitRatio,
            upcomingDoubleSupportSplitRatio, exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);

      Assert.assertEquals("", 4, swingExitCMPProjectionMatrix.numRows);
      Assert.assertEquals("", 1, swingExitCMPProjectionMatrix.numCols);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculation()
   {
      DenseMatrix64F shouldBe = new DenseMatrix64F(4, 1);
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      Random random = new Random();
      int iters = 100;
      double omega0 = 3.0;



      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      SwingExitCMPProjectionMatrix swingExitCMPProjectionMatrix = new SwingExitCMPProjectionMatrix(defaultDoubleSupportSplitRatio,
            upcomingDoubleSupportSplitRatio, exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("currentDoubleSupportDuration", registry));
      doubleSupportDurations.add(new DoubleYoVariable("upcomingDoubleSupportDuration", registry));
      singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration", registry));

      for (int i = 0; i < iters; i++)
      {
         double splitRatio = 0.5 * random.nextDouble();
         double exitRatio = 0.5 * random.nextDouble();

         defaultDoubleSupportSplitRatio.set(splitRatio);
         upcomingDoubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double doubleSupportDuration = 2.0 * random.nextDouble();
         double upcomingDoubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration);
         doubleSupportDurations.get(1).set(upcomingDoubleSupportDuration);
         singleSupportDurations.get(0).set(singleSupportDuration);

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


         String name = "splitRatio = " + splitRatio + ", exitRatio = " + exitRatio + ",\n doubleSupportDuration = " + doubleSupportDuration + ", singleSupportDuration = " + singleSupportDuration;

         double stepDuration = singleSupportDuration + doubleSupportDuration;
         double upcomingInitialDoubleSupport = splitRatio * upcomingDoubleSupportDuration;
         double endOfDoubleSupport = (1.0 - splitRatio) * doubleSupportDuration;

         double timeOnExit = exitRatio * (singleSupportDuration + doubleSupportDuration);
         double timeOnEntry = (1.0 - exitRatio) * (singleSupportDuration + doubleSupportDuration);

         double thirdSegmentTime = singleSupportDuration - endOfSpline;
         double firstSegmentTime = startOfSpline;

         double initialMultiplier = Math.exp(omega0 * (firstSegmentTime + endOfDoubleSupport - timeOnEntry));
         double initialProjection = (1.0 - Math.exp(omega0 * (upcomingInitialDoubleSupport - timeOnExit)));

         boolean useInitialICP = false;

         shouldBe.zero();
         shouldBe.set(0, 0, initialMultiplier * initialProjection);
         shouldBe.set(1, 0, omega0 * initialMultiplier * initialProjection);
         shouldBe.set(2, 0, (1.0 - Math.exp(-omega0 * thirdSegmentTime)));
         shouldBe.set(3, 0, -omega0 * Math.exp(-omega0 * thirdSegmentTime));

         swingExitCMPProjectionMatrix.compute(upcomingDoubleSupportDuration, doubleSupportDuration, singleSupportDuration, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingExitCMPProjectionMatrix, epsilon);
         swingExitCMPProjectionMatrix.reset();
         swingExitCMPProjectionMatrix.compute(doubleSupportDurations, singleSupportDurations, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingExitCMPProjectionMatrix, epsilon);



         useInitialICP = true;

         shouldBe.zero();
         shouldBe.set(2, 0, (1.0 - Math.exp(-omega0 * thirdSegmentTime)));
         shouldBe.set(3, 0, -omega0 * Math.exp(-omega0 * thirdSegmentTime));

         swingExitCMPProjectionMatrix.compute(upcomingDoubleSupportDuration, doubleSupportDuration, singleSupportDuration, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingExitCMPProjectionMatrix, epsilon);
         swingExitCMPProjectionMatrix.reset();
         swingExitCMPProjectionMatrix.compute(doubleSupportDurations, singleSupportDurations, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingExitCMPProjectionMatrix, epsilon);

         shouldBe.zero();
         swingExitCMPProjectionMatrix.reset();
         JUnitTools.assertMatrixEquals(name, shouldBe, swingExitCMPProjectionMatrix, epsilon);
      }
   }

   public static void main(String[] args)
   {
      MutationTestingTools.doPITMutationTestAndOpenResult(SwingExitCMPProjectionMatrixTest.class);
   }
}
