package us.ihmc.math.linearDynamicSysems;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.math.EigenvalueDecomposer;
import us.ihmc.math.ComplexNumber;
import us.ihmc.math.linearDynamicSystems.LinearDynamicSystem;

import static org.junit.jupiter.api.Assertions.*;

public class MassSpringDamperTest
{
   private static final double wn = 1.0;
   private static final double zeta = 0.5;
   private static final double P1 = zeta * wn;
   private static final double P2 = Math.sqrt(1.0 - zeta * zeta) * wn;
   @SuppressWarnings("unused")
   private static final double P3 = wn * wn;

   private SimpleMatrix matrixA;
   private ComplexNumber[] leftEigenvectorV, rightEigenvectorW;
   private ComplexNumber[] eigenvalues;

   private EigenvalueDecomposer eigenvalueDecomposerA;

   @BeforeEach
   public void setUp()
   {
      matrixA = new SimpleMatrix(new double[][]
      {
         {-2.0 * zeta * wn, -wn * wn}, {1.0, 0.0}
      });

      leftEigenvectorV = new ComplexNumber[] {new ComplexNumber(-P2, -P1), new ComplexNumber(0.0, 1.0)};
      rightEigenvectorW = new ComplexNumber[] {new ComplexNumber(-1.0 / (2.0 * P2), 0.0), new ComplexNumber(-P1 / (2.0 * P2), -0.5)};

      eigenvalues = new ComplexNumber[] {new ComplexNumber(-P1, P2), new ComplexNumber(-P1, -P2)};

      eigenvalueDecomposerA = new EigenvalueDecomposer(matrixA);

      new LinearDynamicSystem(matrixA, null, null, null);
   }

   @AfterEach
   public void tearDown()
   {
      matrixA = null;
      leftEigenvectorV = null;
      rightEigenvectorW = null;

      eigenvalues = null;

      eigenvalueDecomposerA = null;
   }

	@Test
   public void testVDotWEqualsOne()
   {
      ComplexNumber dotProduct = new ComplexNumber(0.0, 0.0);
      for (int i = 0; i < leftEigenvectorV.length; i++)
      {
         dotProduct = dotProduct.plus(leftEigenvectorV[i].times(rightEigenvectorW[i]));
      }

      assertEquals(1.0, dotProduct.real(), 1e-7);
      assertEquals(0.0, dotProduct.imaginary(), 1e-7);
   }

	@Test
   public void testVDotWCongEqualsZero()
   {
      ComplexNumber dotProduct = new ComplexNumber(0.0, 0.0);
      for (int i = 0; i < leftEigenvectorV.length; i++)
      {
         dotProduct = dotProduct.plus(leftEigenvectorV[i].times(rightEigenvectorW[i].conjugate()));
      }

      assertEquals(0.0, dotProduct.real(), 1e-7);
      assertEquals(0.0, dotProduct.imaginary(), 1e-7);
   }

	@Test
   public void testDecomposerGotEigenvaluesRight()
   {
      ComplexNumber[] eigenvaluesFromDecomposer = eigenvalueDecomposerA.getEigenvalues();
      DynamicSystemsTestHelpers.assertEpsilonEquals(eigenvalues, eigenvaluesFromDecomposer, 1e-7);
   }
}
