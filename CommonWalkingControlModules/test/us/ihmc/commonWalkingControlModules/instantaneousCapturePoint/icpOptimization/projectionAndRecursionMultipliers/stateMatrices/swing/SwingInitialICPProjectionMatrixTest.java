package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.MutationTestingTools;

import java.util.ArrayList;
import java.util.Random;

public class SwingInitialICPProjectionMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);

      SwingInitialICPProjectionMatrix swingStateEndRecursionMatrix = new SwingInitialICPProjectionMatrix(startOfSplineTime);

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



      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);

      SwingInitialICPProjectionMatrix swingStateEndRecursionMatrix = new SwingInitialICPProjectionMatrix(startOfSplineTime);

      for (int i = 0; i < iters; i++)
      {
         double singleSupportDuration = 5.0 * random.nextDouble();

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupportDuration;

         startOfSplineTime.set(startOfSpline);

         String name = " = " ;

         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(omega0 * startOfSpline));
         shouldBe.set(1, 0, omega0 * Math.exp(omega0 * startOfSpline));

         swingStateEndRecursionMatrix.compute(omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingStateEndRecursionMatrix, epsilon);

         swingStateEndRecursionMatrix.reset();
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, swingStateEndRecursionMatrix, epsilon);
      }
   }

   public static void main(String[] args)
   {
      MutationTestingTools.doPITMutationTestAndOpenResult(SwingInitialICPProjectionMatrixTest.class);
   }
}
