package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing.SwingStateEndRecursionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.MutationTestingTools;

import java.util.ArrayList;
import java.util.Random;

public class SwingStateEndRecursionMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      SwingStateEndRecursionMatrix swingStateEndRecursionMatrix = new SwingStateEndRecursionMatrix(defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            startOfSplineTime, endOfSplineTime, totalTrajectoryTime);

      Assert.assertEquals("", 4, swingStateEndRecursionMatrix.numRows);
      Assert.assertEquals("", 1, swingStateEndRecursionMatrix.numCols);
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
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("currentDoubleSupportDuration", registry));
      doubleSupportDurations.add(new DoubleYoVariable("upcomingDoubleSupportDuration", registry));
      singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration", registry));


      SwingStateEndRecursionMatrix swingStateEndRecursionMatrix = new SwingStateEndRecursionMatrix(defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            startOfSplineTime, endOfSplineTime, totalTrajectoryTime);

      for (int i = 0; i < iters; i++)
      {
         double splitRatio = 0.5 * random.nextDouble();

         defaultDoubleSupportSplitRatio.set(splitRatio);
         upcomingDoubleSupportSplitRatio.set(splitRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double upcomingDoubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         doubleSupportDurations.get(1).set(upcomingDoubleSupportDuration);
         singleSupportDurations.get(0).set(singleSupportDuration);

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupportDuration;
         double endOfSpline = Math.max(alpha1, alpha2) * singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         double stepDuration = currentDoubleSupportDuration + singleSupportDuration;
         double upcomingInitialDoubleSupportDuration = splitRatio * upcomingDoubleSupportDuration;
         double endOfDoubleSupportDuration = (1.0 - splitRatio) * currentDoubleSupportDuration;

         String name = "splitRatio = " + splitRatio + ",\n doubleSupportDuration = " + currentDoubleSupportDuration + ", singleSupportDuration = " + singleSupportDuration;

         double recursionTime = upcomingInitialDoubleSupportDuration + endOfDoubleSupportDuration + startOfSpline - stepDuration;
         double lastSegmentDuration = singleSupportDuration - endOfSpline;

         boolean useInitialICP = false;

         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(omega0 * recursionTime));
         shouldBe.set(1, 0, omega0 * Math.exp(omega0 * recursionTime));
         shouldBe.set(2, 0, Math.exp(-omega0 * lastSegmentDuration));
         shouldBe.set(3, 0, omega0 * Math.exp(-omega0 * lastSegmentDuration));

         swingStateEndRecursionMatrix.compute(upcomingDoubleSupportDuration, currentDoubleSupportDuration, singleSupportDuration, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingStateEndRecursionMatrix, epsilon);
         swingStateEndRecursionMatrix.reset();
         swingStateEndRecursionMatrix.compute(doubleSupportDurations, singleSupportDurations, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingStateEndRecursionMatrix, epsilon);


         useInitialICP = true;

         shouldBe.zero();
         shouldBe.set(2, 0, Math.exp(-omega0 * lastSegmentDuration));
         shouldBe.set(3, 0, omega0 * Math.exp(-omega0 * lastSegmentDuration));

         swingStateEndRecursionMatrix.compute(upcomingDoubleSupportDuration, currentDoubleSupportDuration, singleSupportDuration, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingStateEndRecursionMatrix, epsilon);
         swingStateEndRecursionMatrix.reset();
         swingStateEndRecursionMatrix.compute(doubleSupportDurations, singleSupportDurations, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingStateEndRecursionMatrix, epsilon);

         swingStateEndRecursionMatrix.reset();
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, swingStateEndRecursionMatrix, epsilon);
      }
   }

   public static void main(String[] args)
   {
      MutationTestingTools.doPITMutationTestAndOpenResult(SwingStateEndRecursionMatrixTest.class);
   }
}
