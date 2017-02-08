package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.NewSwingInitialICPMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;

import java.util.ArrayList;
import java.util.Random;

public class NewTransferInitialICPVelocityMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      NewTransferInitialICPVelocityMatrix initialICPVelocityMatrix = new NewTransferInitialICPVelocityMatrix();

      Assert.assertEquals("", 4, initialICPVelocityMatrix.numRows);
      Assert.assertEquals("", 1, initialICPVelocityMatrix.numCols);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculation()
   {
      DenseMatrix64F shouldBe = new DenseMatrix64F(4, 1);
      int iters = 100;

      NewTransferInitialICPVelocityMatrix initialICPVelocityMatrix = new NewTransferInitialICPVelocityMatrix();

      for (int i = 0; i < iters; i++)
      {
         initialICPVelocityMatrix.compute();

         shouldBe.zero();
         shouldBe.set(0, 0, 0.0);
         shouldBe.set(1, 0, 1.0);

         JUnitTools.assertMatrixEquals("", shouldBe, initialICPVelocityMatrix, epsilon);

         shouldBe.zero();
         initialICPVelocityMatrix.reset();
         JUnitTools.assertMatrixEquals("", shouldBe, initialICPVelocityMatrix, epsilon);
      }
   }
}
