package us.ihmc.robotics.linearDynamicSystems;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import Jama.Matrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.Polynomial;

public class PolynomialMatrixTest
{
   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeDeterminantOne()
   {
      Polynomial zero = new Polynomial(new double[] {0.0});
      Polynomial one = new Polynomial(new double[] {1.0});
      Polynomial two = new Polynomial(new double[] {2.0});

      Polynomial[][] polynomials = new Polynomial[][]
      {
         {one}
      };
      PolynomialMatrix m0 = new PolynomialMatrix(polynomials);
      assertTrue(m0.computeDeterminant().epsilonEquals(one, 1e-7));

      polynomials = new Polynomial[][]
      {
         {one, one}, {zero, one}
      };
      m0 = new PolynomialMatrix(polynomials);
      Polynomial determinant = m0.computeDeterminant();
      assertTrue(determinant.epsilonEquals(one, 1e-7));

      polynomials = new Polynomial[][]
      {
         {one, two}, {two, two}
      };
      m0 = new PolynomialMatrix(polynomials);
      determinant = m0.computeDeterminant();
      assertTrue(determinant.epsilonEquals(two.times(-1.0), 1e-7));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeDeterminantTwo()
   {
      // From J.Pratt MCSI Problem Set 5.

      Polynomial s = new Polynomial(new double[] {1.0, 0.0});
      Polynomial zero = new Polynomial(new double[] {0.0});
      Polynomial one = new Polynomial(new double[] {1.0});
      Polynomial negativeOne = one.times(-1.0);

      Polynomial[][] polynomials = new Polynomial[][]
      {
         {s, zero, negativeOne, zero}, {zero, s, zero, negativeOne}, {one, negativeOne, s, zero}, {negativeOne, one, zero, s}
      };

      Polynomial expectedDeterminant = new Polynomial(new double[] {1.0, 0.0, 2.0, 0.0, 0.0});
      PolynomialMatrix m0 = new PolynomialMatrix(polynomials);
      Polynomial determinant = m0.computeDeterminant();

      assertTrue(determinant.epsilonEquals(expectedDeterminant, 1e-7));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeDeterminantAndConstructSIMinusA()
   {
      Matrix matrixA = new Matrix(new double[][]
      {
         {1.0}
      });

      PolynomialMatrix sIMinusA = PolynomialMatrix.constructSIMinusA(matrixA);
      Polynomial determinant = sIMinusA.computeDeterminant();

      Polynomial expectedPolynomial = new Polynomial(new double[] {1.0, -1.0});
      assertTrue(expectedPolynomial.epsilonEquals(determinant, 1e-7));

      // From J.Pratt MCSI Problem Set 5.

      matrixA = new Matrix(new double[][]
      {
         {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}, {-1.0, 1.0, 0.0, 0.0}, {1.0, -1.0, 0.0, 0.0}
      });

      sIMinusA = PolynomialMatrix.constructSIMinusA(matrixA);
      determinant = sIMinusA.computeDeterminant();

      expectedPolynomial = new Polynomial(new double[] {1.0, 0.0, 2.0, 0.0, 0.0});
      assertTrue(expectedPolynomial.epsilonEquals(determinant, 1e-7));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeDeterminantAndCofactors()
   {
      // From J.Pratt MCSI Problem Set 3.

      double[][] elementsA = new double[][]
      {
         {2.0, -2.0, 3.0}, {1.0, 1.0, 1.0}, {1.0, 3.0, -1.0}
      };
      Matrix matrixA = new Matrix(elementsA);

      PolynomialMatrix sIMinusA = PolynomialMatrix.constructSIMinusA(matrixA);
      Polynomial determinant = sIMinusA.computeDeterminant();

      Polynomial expectedPolynomial =
         Polynomial.constructFromRealRoot(1.0).times(Polynomial.constructFromRealRoot(-2.0)).times(Polynomial.constructFromRealRoot(3.0));
      assertTrue(expectedPolynomial.epsilonEquals(determinant, 1e-7));

      Polynomial[][] cofactors = sIMinusA.computeCofactors();


      assertTrue(cofactors[0][0].epsilonEquals(new Polynomial(1.0, 0.0, -4.0), 1e-7));
      assertTrue(cofactors[0][1].epsilonEquals(new Polynomial(1.0, 2.0), 1e-7));
      assertTrue(cofactors[0][2].epsilonEquals(new Polynomial(1.0, 2.0), 1e-7));

      assertTrue(cofactors[1][0].epsilonEquals(new Polynomial(-2.0, 7.0), 1e-7));
      assertTrue(cofactors[1][1].epsilonEquals(new Polynomial(1.0, -1.0, -5.0), 1e-7));
      assertTrue(cofactors[1][2].epsilonEquals(new Polynomial(3.0, -8.0), 1e-7));

      assertTrue(cofactors[2][0].epsilonEquals(new Polynomial(3.0, -5.0), 1e-7));
      assertTrue(cofactors[2][1].epsilonEquals(new Polynomial(1.0, 1.0), 1e-7));
      assertTrue(cofactors[2][2].epsilonEquals(new Polynomial(1.0, -3.0, +4.0), 1e-7));

   }

}
