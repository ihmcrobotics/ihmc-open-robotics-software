package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing.SwingEntryCMPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

public class SwingEntryCMPProjectionMatrixTest
{
   private static final double epsilon = 0.00001;

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);

      SwingEntryCMPProjectionMatrix swingEntryCMPProjectionMatrix = new SwingEntryCMPProjectionMatrix(doubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime);

      Assert.assertEquals("", 4, swingEntryCMPProjectionMatrix.numRows);
      Assert.assertEquals("", 1, swingEntryCMPProjectionMatrix.numCols);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculation()
   {
      DenseMatrix64F shouldBe = new DenseMatrix64F(4, 1);
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      Random random = new Random();
      int iters = 100;

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);

      SwingEntryCMPProjectionMatrix swingEntryCMPProjectionMatrix = new SwingEntryCMPProjectionMatrix(doubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime);

      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.5 * random.nextDouble();
         double exitRatio = 0.5 * random.nextDouble();
         double startOfSpline = 0.2 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);
         startOfSplineTime.set(startOfSpline);

         double doubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();

         String name = "splitRatio = " + splitRatio + ", exitRatio = " + exitRatio + ",\n doubleSupportDuration = " + doubleSupportDuration + ", singleSupportDuration = " + singleSupportDuration;

         double endOfDoubleSupport = (1.0 - splitRatio) * doubleSupportDuration;
         double timeSpentOnEntry = (1.0 - exitRatio) * (singleSupportDuration + doubleSupportDuration);

         double projectionTime = startOfSpline + endOfDoubleSupport - timeSpentOnEntry;
         double projection = Math.exp(omega0 * projectionTime);

         swingEntryCMPProjectionMatrix.compute(doubleSupportDuration, singleSupportDuration, omega0, false);


         shouldBe.zero();
         shouldBe.set(0, 0, 1.0 - projection);
         shouldBe.set(1, 0, -omega0 * projection);

         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPProjectionMatrix, epsilon);

         projectionTime = startOfSpline;
         projection = Math.exp(omega0 * projectionTime);

         swingEntryCMPProjectionMatrix.compute(doubleSupportDuration, singleSupportDuration, omega0, true);

         shouldBe.zero();
         shouldBe.set(0, 0, 1.0 - projection);
         shouldBe.set(1, 0, -omega0 * projection);

         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPProjectionMatrix, epsilon);
      }
   }
}
