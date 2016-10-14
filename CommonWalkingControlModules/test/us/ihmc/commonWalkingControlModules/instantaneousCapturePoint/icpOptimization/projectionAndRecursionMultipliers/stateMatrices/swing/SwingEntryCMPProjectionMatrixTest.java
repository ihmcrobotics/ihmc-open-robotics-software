package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing.SwingEntryCMPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.MutationTestingTools;

import java.util.ArrayList;
import java.util.Random;

public class SwingEntryCMPProjectionMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);

      SwingEntryCMPProjectionMatrix swingEntryCMPProjectionMatrix = new SwingEntryCMPProjectionMatrix(doubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime);

      Assert.assertEquals("", 4, swingEntryCMPProjectionMatrix.numRows);
      Assert.assertEquals("", 1, swingEntryCMPProjectionMatrix.numCols);
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

      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration", registry));
      singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration", registry));

      SwingEntryCMPProjectionMatrix swingEntryCMPProjectionMatrix = new SwingEntryCMPProjectionMatrix(doubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime);

      for (int i = 0; i < iters; i++)
      {
         double splitRatio = 0.5 * random.nextDouble();
         double exitRatio = 0.5 * random.nextDouble();
         double startOfSpline = 0.2 * random.nextDouble();

         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);
         startOfSplineTime.set(startOfSpline);

         double doubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration);
         singleSupportDurations.get(0).set(singleSupportDuration);

         String name = "splitRatio = " + splitRatio + ", exitRatio = " + exitRatio + ",\n doubleSupportDuration = " + doubleSupportDuration + ", singleSupportDuration = " + singleSupportDuration;

         double endOfDoubleSupport = (1.0 - splitRatio) * doubleSupportDuration;
         double timeSpentOnEntry = (1.0 - exitRatio) * (singleSupportDuration + doubleSupportDuration);

         double projectionTime = startOfSpline + endOfDoubleSupport - timeSpentOnEntry;
         double projection = Math.exp(omega0 * projectionTime);

         boolean useInitialICP = false;
         swingEntryCMPProjectionMatrix.compute(doubleSupportDuration, singleSupportDuration, omega0, useInitialICP);
         shouldBe.zero();
         shouldBe.set(0, 0, 1.0 - projection);
         shouldBe.set(1, 0, -omega0 * projection);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPProjectionMatrix, epsilon);
         swingEntryCMPProjectionMatrix.reset();
         swingEntryCMPProjectionMatrix.compute(doubleSupportDurations, singleSupportDurations, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPProjectionMatrix, epsilon);

         useInitialICP = true;
         projectionTime = startOfSpline;
         projection = Math.exp(omega0 * projectionTime);

         swingEntryCMPProjectionMatrix.compute(doubleSupportDuration, singleSupportDuration, omega0, useInitialICP);

         shouldBe.zero();
         shouldBe.set(0, 0, 1.0 - projection);
         shouldBe.set(1, 0, -omega0 * projection);

         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPProjectionMatrix, epsilon);
         swingEntryCMPProjectionMatrix.reset();
         swingEntryCMPProjectionMatrix.compute(doubleSupportDurations, singleSupportDurations, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPProjectionMatrix, epsilon);

         shouldBe.zero();
         swingEntryCMPProjectionMatrix.reset();
         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPProjectionMatrix, epsilon);
      }
   }

   public static void main(String[] args)
   {
      MutationTestingTools.doPITMutationTestAndOpenResult(SwingEntryCMPProjectionMatrixTest.class);
   }
}
