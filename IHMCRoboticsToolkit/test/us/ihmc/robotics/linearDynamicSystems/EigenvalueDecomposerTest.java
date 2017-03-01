package us.ihmc.robotics.linearDynamicSystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import Jama.Matrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.dataStructures.ComplexNumber;

public class EigenvalueDecomposerTest
{
   // Parameters for mass-spring-damper:
   private static final double
   wn = 1.0, zeta = 0.5;
   private static final double
   P1 = zeta * wn, P2 = Math.sqrt(1.0 - zeta * zeta) * wn, P3 = wn * wn;
   private double epsilon = 1e-7;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetEigenvalues()
   {

      Matrix matrixAOneReal = new Matrix(new double[][]
            {
            {1.0}
            });
      Matrix matrixBTwoComplex = new Matrix(new double[][]
            {
            {3.0, -2.0}, {4.0, -1.0}
            });
      Matrix matrixCOneRealTwoComplex = new Matrix(new double[][]
            {
            {3.0, -2.0, 0.0}, {4.0, -1.0, 0.0}, {0.0, 0.0, -5.0}
            });
      Matrix matrixMassSpringDamper = new Matrix(new double[][]
            {
            {-2.0 * P1, -P3}, {1.0, 0.0}
            });

      EigenvalueDecomposer decomposerAOneReal = new EigenvalueDecomposer(matrixAOneReal);
      EigenvalueDecomposer decomposerBTwoComplex = new EigenvalueDecomposer(matrixBTwoComplex);
      EigenvalueDecomposer decomposerCOneRealTwoComplex = new EigenvalueDecomposer(matrixCOneRealTwoComplex);
      EigenvalueDecomposer decomposerMassSpringDamper = new EigenvalueDecomposer(matrixMassSpringDamper);

      verifyOneRealEigenvalue(decomposerAOneReal.getEigenvalues(), 1.0);
      verifyTwoComplexConjugateEigenvalue(decomposerBTwoComplex.getEigenvalues(), 1.0, 2.0);
      verifyOneRealTwoComplexConjugateEigenvalue(decomposerCOneRealTwoComplex.getEigenvalues(), -5.0, 1.0, 2.0);
      verifyTwoComplexConjugateEigenvalue(decomposerMassSpringDamper.getEigenvalues(), -P1, P2);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDecompositions()
   {
      Matrix matrixAOneReal = new Matrix(new double[][]
            {
            {1.0}
            });
      Matrix matrixBTwoComplex = new Matrix(new double[][]
            {
            {3.0, -2.0}, {4.0, -1.0}
            });
      Matrix matrixCOneRealTwoComplex = new Matrix(new double[][]
            {
            {3.0, -2.0, 0.0}, {4.0, -1.0, 0.0}, {0.0, 0.0, -5.0}
            });
      Matrix matrixMassSpringDamper = new Matrix(new double[][]
            {
            {-2.0 * P1, -P3}, {1.0, 0.0}
            });

      EigenvalueDecomposer decomposerAOneReal = new EigenvalueDecomposer(matrixAOneReal);
      EigenvalueDecomposer decomposerBTwoComplex = new EigenvalueDecomposer(matrixBTwoComplex);
      EigenvalueDecomposer decomposerCOneRealTwoComplex = new EigenvalueDecomposer(matrixCOneRealTwoComplex);
      EigenvalueDecomposer decomposerMassSpringDamper = new EigenvalueDecomposer(matrixMassSpringDamper);

      verifyDecomposition(matrixAOneReal, decomposerAOneReal);
      verifyDecomposition(matrixBTwoComplex, decomposerBTwoComplex);
      verifyDecomposition(matrixCOneRealTwoComplex, decomposerCOneRealTwoComplex);
      verifyDecomposition(matrixMassSpringDamper, decomposerMassSpringDamper);
   }

   private void verifyOneRealEigenvalue(ComplexNumber[] eigenvalues, double real)
   {
      assertEquals(1, eigenvalues.length);
      assertEquals(real, eigenvalues[0].real(), 1e-7);
      assertEquals(0.0, eigenvalues[0].imag(), epsilon);
   }

   private void verifyTwoComplexConjugateEigenvalue(ComplexNumber[] eigenvalues, double real, double imag)
   {
      assertEquals(2, eigenvalues.length);
      verifyTwoComplexConjugateEigenvalue(eigenvalues[0], eigenvalues[1], real, imag);
   }

   private void verifyTwoComplexConjugateEigenvalue(ComplexNumber eigenvalueOne, ComplexNumber eigenvalueTwo, double real, double imag)
   {
      assertEquals(real, eigenvalueOne.real(), 1e-7);
      assertEquals(real, eigenvalueTwo.real(), 1e-7);

      assertEquals(Math.abs(imag), Math.abs(eigenvalueOne.imag()), 1e-7);
      assertEquals(Math.abs(imag), Math.abs(eigenvalueTwo.imag()), 1e-7);
      assertEquals(eigenvalueOne.imag(), -eigenvalueTwo.imag(), 1e-7);
   }

   private void verifyOneRealTwoComplexConjugateEigenvalue(ComplexNumber[] eigenvalues, double oneReal, double realPart, double imagPart)
   {
      assertEquals(3, eigenvalues.length);

      boolean foundRealOne = false, foundComplexPair = false;

      int index = 0;
      while (index < eigenvalues.length)
      {
         ComplexNumber eigenvalue = eigenvalues[index];

         if (eigenvalue.imag() == 0.0)
         {
            assertFalse(foundRealOne);
            assertEquals(oneReal, eigenvalue.real(), 1e-7);
            foundRealOne = true;
            index++;
         }
         else
         {
            assertFalse(foundComplexPair);
            ComplexNumber eigenvalueTwo = eigenvalues[index + 1];
            verifyTwoComplexConjugateEigenvalue(eigenvalue, eigenvalueTwo, realPart, imagPart);
            foundComplexPair = true;
            index++;
            index++;
         }

      }

      assertTrue(foundRealOne);
      assertTrue(foundComplexPair);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCircleGenerator()
   {
      Matrix matrixA = new Matrix(new double[][]
      {
         {0.0, -1.0}, {1.0, 0.0}
      });

      EigenvalueDecomposer circleDecomposer = new EigenvalueDecomposer(matrixA);

      verifyDecomposition(matrixA, circleDecomposer);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testZeroMatrix()
   {
      Matrix matrixA = new Matrix(new double[][]
      {
         {0.0}
      });
      EigenvalueDecomposer eigenvalueDecomposer = new EigenvalueDecomposer(matrixA);
      verifyDecomposition(matrixA, eigenvalueDecomposer);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIdentityMatrix()
   {
      Matrix matrixA = new Matrix(new double[][]
      {
         {1.0, 0.0},
         {0.0, 1.0}
      });
      EigenvalueDecomposer eigenvalueDecomposer = new EigenvalueDecomposer(matrixA);
      verifyDecomposition(matrixA, eigenvalueDecomposer);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testThreeByThreeIdentityMatrix()
   {
      Matrix matrixA = new Matrix(new double[][]
      {
         {1.0, 0.0, 0.0},
         {0.0, 1.0, 0.0},
         {0.0, 0.0, 1.0}
      });
      EigenvalueDecomposer eigenvalueDecomposer = new EigenvalueDecomposer(matrixA);
      verifyDecomposition(matrixA, eigenvalueDecomposer);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRepeatedPoles()
   {
      Matrix matrixA = new Matrix(new double[][]
      {
         {0.0, 0.0},
         {0.0, 0.0}
      });
      EigenvalueDecomposer eigenvalueDecomposer = new EigenvalueDecomposer(matrixA);
      verifyDecomposition(matrixA, eigenvalueDecomposer);
   }
   
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testMCSExample()
   {
      // From JPratt MSC Notes PSET 5
      // This one has repeat poles at s=0, which makes V non-invertible...

      Matrix matrixA = new Matrix(new double[][]
      {
         {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}, {-1.0, 1.0, 0.0, 0.0}, {1.0, -1.0, 0.0, 0.0}
      });
      EigenvalueDecomposer eigenvalueDecomposer = new EigenvalueDecomposer(matrixA);
      verifyDecomposition(matrixA, eigenvalueDecomposer);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRandomExample()
   {
      // From JPratt MSC Notes PSET 5
      // This one has repeat poles at s=0, which makes V non-invertible...

      Matrix matrixA = new Matrix(new double[][]
      {
         {1.0, 5.0, 1.0, 7.0}, {0.0, 3.0, 0.0, 1.0}, {-1.0, 1.0, 0.0, 9.0}, {1.0, -1.0, 0.0, 0.0}
      });
      EigenvalueDecomposer eigenvalueDecomposer = new EigenvalueDecomposer(matrixA);
      verifyDecomposition(matrixA, eigenvalueDecomposer);
   }

   private static void verifyDecomposition(Matrix matrix, EigenvalueDecomposer decomposer)
   {
      verifyWEqualsVInverse(decomposer);
      verifyAEqualsVLambdaW(matrix, decomposer);
   }

   private static void verifyWEqualsVInverse(EigenvalueDecomposer decomposer)
   {
      ComplexNumber[][] leftEigenvectors = decomposer.getLeftEigenvectors();
      ComplexNumber[][] rightEigenvectors = decomposer.getRightEigenvectors();

      ComplexMatrix complexV = new ComplexMatrix(leftEigenvectors);
      complexV.transpose();

      ComplexMatrix complexW = new ComplexMatrix(rightEigenvectors);

      ComplexMatrix identityReconstructed = complexV.times(complexW);
      ComplexMatrix identity = ComplexMatrix.constructIdentity(leftEigenvectors.length);

      identityReconstructed.epsilonEquals(identity, 1e-7);
   }


   private static void verifyAEqualsVLambdaW(Matrix matrixA, EigenvalueDecomposer decomposer)
   {
      ComplexNumber[] eigenvalues = decomposer.getEigenvalues();
      ComplexNumber[][] leftEigenvectors = decomposer.getLeftEigenvectors();
      ComplexNumber[][] rightEigenvectors = decomposer.getRightEigenvectors();

      ComplexMatrix complexV = new ComplexMatrix(leftEigenvectors);
      complexV = complexV.transpose();

      ComplexMatrix complexW = new ComplexMatrix(rightEigenvectors);

      ComplexMatrix lambda = ComplexMatrix.constructDiagonalMatrix(eigenvalues);

      System.out.println("V = " + complexV);
      System.out.println("lambda = " + lambda);
      System.out.println("W = " + complexW);

      ComplexMatrix aReconstructed = complexV.times(lambda).times(complexW);

      boolean passed = aReconstructed.epsilonEquals(matrixA, 1e-7);

      if (!passed)
      {
         DynamicSystemsTestHelpers.printMatrix("matrixA: ", matrixA);
         System.out.println("\naReconstructed: \n" + aReconstructed);
         
         System.out.println("V = " + complexV);
         System.out.println("W = " + complexW);
         System.out.println("lambda = " + lambda);
      }

      assertTrue(passed);
   }
}
