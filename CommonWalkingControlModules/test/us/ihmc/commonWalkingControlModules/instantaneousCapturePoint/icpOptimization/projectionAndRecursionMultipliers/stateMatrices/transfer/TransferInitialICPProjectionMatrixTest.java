package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

public class TransferInitialICPProjectionMatrixTest
{
   private static final double epsilon = 0.00001;

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);

      TransferInitialICPProjectionMatrix transferInitialICPProjectionMatrix = new TransferInitialICPProjectionMatrix();

      Assert.assertEquals("", 4, transferInitialICPProjectionMatrix.numRows);
      Assert.assertEquals("", 1, transferInitialICPProjectionMatrix.numCols);
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

      TransferInitialICPProjectionMatrix transferInitialICPProjectionMatrix = new TransferInitialICPProjectionMatrix();

      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.5 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();

         String name = "splitRatio = " + splitRatio + ",\n doubleSupportDuration = " + currentDoubleSupportDuration + ", singleSupportDuration = " + singleSupportDuration;


         transferInitialICPProjectionMatrix.compute(omega0);
         shouldBe.zero();
         shouldBe.set(0, 0, 1.0);
         shouldBe.set(1, 0, omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferInitialICPProjectionMatrix, epsilon);
      }
   }
}
