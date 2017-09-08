package us.ihmc.robotics.linearDynamicSystems;

import static org.junit.Assert.assertEquals;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import Jama.Matrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.ComplexNumber;

public class MassSpringDamperTest
{
   private static final double wn = 1.0;
   private static final double zeta = 0.5;
   private static final double P1 = zeta * wn;
   private static final double P2 = Math.sqrt(1.0 - zeta * zeta) * wn;
   @SuppressWarnings("unused")
   private static final double P3 = wn * wn;

   private Matrix matrixA;
   private ComplexNumber[] leftEigenvectorV, rightEigenvectorW;
   private ComplexNumber[] eigenvalues;

   private EigenvalueDecomposer eigenvalueDecomposerA;

   @Before
   public void setUp() throws Exception
   {
      matrixA = new Matrix(new double[][]
      {
         {-2.0 * zeta * wn, -wn * wn}, {1.0, 0.0}
      });

      leftEigenvectorV = new ComplexNumber[] {new ComplexNumber(-P2, -P1), new ComplexNumber(0.0, 1.0)};
      rightEigenvectorW = new ComplexNumber[] {new ComplexNumber(-1.0 / (2.0 * P2), 0.0), new ComplexNumber(-P1 / (2.0 * P2), -0.5)};

      eigenvalues = new ComplexNumber[] {new ComplexNumber(-P1, P2), new ComplexNumber(-P1, -P2)};

      eigenvalueDecomposerA = new EigenvalueDecomposer(matrixA);

      new LinearDynamicSystem(matrixA, null, null, null);
   }

   @After
   public void tearDown() throws Exception
   {
      matrixA = null;
      leftEigenvectorV = null;
      rightEigenvectorW = null;

      eigenvalues = null;

      eigenvalueDecomposerA = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testVDotWEqualsOne()
   {
      ComplexNumber dotProduct = new ComplexNumber(0.0, 0.0);
      for (int i = 0; i < leftEigenvectorV.length; i++)
      {
         dotProduct = dotProduct.plus(leftEigenvectorV[i].times(rightEigenvectorW[i]));
      }

      assertEquals(1.0, dotProduct.real(), 1e-7);
      assertEquals(0.0, dotProduct.imag(), 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testVDotWCongEqualsZero()
   {
      ComplexNumber dotProduct = new ComplexNumber(0.0, 0.0);
      for (int i = 0; i < leftEigenvectorV.length; i++)
      {
         dotProduct = dotProduct.plus(leftEigenvectorV[i].times(rightEigenvectorW[i].conj()));
      }

      assertEquals(0.0, dotProduct.real(), 1e-7);
      assertEquals(0.0, dotProduct.imag(), 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDecomposerGotEigenvaluesRight()
   {
      ComplexNumber[] eigenvaluesFromDecomposer = eigenvalueDecomposerA.getEigenvalues();
      DynamicSystemsTestHelpers.assertEpsilonEquals(eigenvalues, eigenvaluesFromDecomposer, 1e-7);
   }

//   @Test(timeout=300000)
//   public void testDynamicSystem()
//   {
//      @SuppressWarnings("unused") TransferFunctionMatrix transferFunctionMatrix = dynamicSystem.getTransferFunctionMatrix();
//
//      fail("Finish Implementing!");
//   }



}
