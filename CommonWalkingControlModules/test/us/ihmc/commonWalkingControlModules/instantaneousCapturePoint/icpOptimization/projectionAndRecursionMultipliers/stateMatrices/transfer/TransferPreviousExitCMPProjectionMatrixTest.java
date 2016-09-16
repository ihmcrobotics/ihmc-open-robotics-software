package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer.TransferPreviousExitCMPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

public class TransferPreviousExitCMPProjectionMatrixTest
{
   private static final double epsilon = 0.00001;

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      TransferPreviousExitCMPProjectionMatrix entryCMPProjectionMatrix = new TransferPreviousExitCMPProjectionMatrix(doubleSupportSplitRatio);

      Assert.assertEquals("", 4, entryCMPProjectionMatrix.numRows);
      Assert.assertEquals("", 1, entryCMPProjectionMatrix.numCols);
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

      TransferPreviousExitCMPProjectionMatrix entryCMPProjectionMatrix = new TransferPreviousExitCMPProjectionMatrix(doubleSupportSplitRatio);

      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.5 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);

         double doubleSupportDuration = 2.0 * random.nextDouble();

         String name = "splitRatio = " + splitRatio + ", doubleSupportDuration = " + doubleSupportDuration;

         double initialDoubleSupport = splitRatio * doubleSupportDuration;

         boolean useInitialICP = false;

         entryCMPProjectionMatrix.compute(doubleSupportDuration, omega0, useInitialICP);
         shouldBe.zero();
         shouldBe.set(0, 0, 1.0 - Math.exp(-omega0 * initialDoubleSupport));
         shouldBe.set(1, 0, -omega0 * Math.exp(-omega0 * initialDoubleSupport));

         JUnitTools.assertMatrixEquals(name, shouldBe, entryCMPProjectionMatrix, epsilon);

         useInitialICP = true;

         entryCMPProjectionMatrix.compute(doubleSupportDuration, omega0, useInitialICP);
         shouldBe.zero();
         shouldBe.set(1, 0, -omega0);

         JUnitTools.assertMatrixEquals(name, shouldBe, entryCMPProjectionMatrix, epsilon);
      }
   }
}
