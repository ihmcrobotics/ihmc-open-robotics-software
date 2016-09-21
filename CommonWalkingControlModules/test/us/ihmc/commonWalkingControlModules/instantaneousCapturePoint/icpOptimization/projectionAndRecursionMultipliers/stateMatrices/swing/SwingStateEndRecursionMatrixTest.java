package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing.SwingStateEndRecursionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

public class SwingStateEndRecursionMatrixTest
{
   private static final double epsilon = 0.00001;

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      SwingStateEndRecursionMatrix swingStateEndRecursionMatrix = new SwingStateEndRecursionMatrix(doubleSupportSplitRatio, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);

      Assert.assertEquals("", 4, swingStateEndRecursionMatrix.numRows);
      Assert.assertEquals("", 1, swingStateEndRecursionMatrix.numCols);
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
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      SwingStateEndRecursionMatrix swingStateEndRecursionMatrix = new SwingStateEndRecursionMatrix(doubleSupportSplitRatio, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);

      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.5 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double upcomingDoubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();

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

         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(omega0 * recursionTime));
         shouldBe.set(1, 0, omega0 * Math.exp(omega0 * recursionTime));
         shouldBe.set(2, 0, Math.exp(-omega0 * lastSegmentDuration));
         shouldBe.set(3, 0, omega0 * Math.exp(-omega0 * lastSegmentDuration));

         swingStateEndRecursionMatrix.compute(upcomingDoubleSupportDuration, currentDoubleSupportDuration, singleSupportDuration, omega0, false);

         JUnitTools.assertMatrixEquals(name, shouldBe, swingStateEndRecursionMatrix, epsilon);



         shouldBe.zero();
         shouldBe.set(2, 0, Math.exp(-omega0 * lastSegmentDuration));
         shouldBe.set(3, 0, omega0 * Math.exp(-omega0 * lastSegmentDuration));

         swingStateEndRecursionMatrix.compute(upcomingDoubleSupportDuration, currentDoubleSupportDuration, singleSupportDuration, omega0, true);

         JUnitTools.assertMatrixEquals(name, shouldBe, swingStateEndRecursionMatrix, epsilon);
      }
   }
}
