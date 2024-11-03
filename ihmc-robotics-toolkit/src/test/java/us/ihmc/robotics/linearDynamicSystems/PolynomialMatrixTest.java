package us.ihmc.robotics.linearDynamicSystems;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import Jama.Matrix;
import us.ihmc.robotics.dataStructures.ObsoletePolynomial;

public class PolynomialMatrixTest
{
   @BeforeEach
   public void setUp() throws Exception
   {
   }

   @AfterEach
   public void tearDown() throws Exception
   {
   }

	@Test
   public void testComputeDeterminantOne()
   {
      ObsoletePolynomial zero = new ObsoletePolynomial(new double[] {0.0});
      ObsoletePolynomial one = new ObsoletePolynomial(new double[] {1.0});
      ObsoletePolynomial two = new ObsoletePolynomial(new double[] {2.0});

      ObsoletePolynomial[][] polynomials = new ObsoletePolynomial[][]
      {
         {one}
      };
      PolynomialMatrix m0 = new PolynomialMatrix(polynomials);
      assertTrue(m0.computeDeterminant().epsilonEquals(one, 1e-7));

      polynomials = new ObsoletePolynomial[][]
      {
         {one, one}, {zero, one}
      };
      m0 = new PolynomialMatrix(polynomials);
      ObsoletePolynomial determinant = m0.computeDeterminant();
      assertTrue(determinant.epsilonEquals(one, 1e-7));

      polynomials = new ObsoletePolynomial[][]
      {
         {one, two}, {two, two}
      };
      m0 = new PolynomialMatrix(polynomials);
      determinant = m0.computeDeterminant();
      assertTrue(determinant.epsilonEquals(two.times(-1.0), 1e-7));
   }

	@Test
   public void testComputeDeterminantTwo()
   {
      // From J.Pratt MCSI Problem Set 5.

      ObsoletePolynomial s = new ObsoletePolynomial(new double[] {1.0, 0.0});
      ObsoletePolynomial zero = new ObsoletePolynomial(new double[] {0.0});
      ObsoletePolynomial one = new ObsoletePolynomial(new double[] {1.0});
      ObsoletePolynomial negativeOne = one.times(-1.0);

      ObsoletePolynomial[][] polynomials = new ObsoletePolynomial[][]
      {
         {s, zero, negativeOne, zero}, {zero, s, zero, negativeOne}, {one, negativeOne, s, zero}, {negativeOne, one, zero, s}
      };

      ObsoletePolynomial expectedDeterminant = new ObsoletePolynomial(new double[] {1.0, 0.0, 2.0, 0.0, 0.0});
      PolynomialMatrix m0 = new PolynomialMatrix(polynomials);
      ObsoletePolynomial determinant = m0.computeDeterminant();

      assertTrue(determinant.epsilonEquals(expectedDeterminant, 1e-7));
   }

	@Test
   public void testComputeDeterminantAndConstructSIMinusA()
   {
      Matrix matrixA = new Matrix(new double[][]
      {
         {1.0}
      });

      PolynomialMatrix sIMinusA = PolynomialMatrix.constructSIMinusA(matrixA);
      ObsoletePolynomial determinant = sIMinusA.computeDeterminant();

      ObsoletePolynomial expectedPolynomial = new ObsoletePolynomial(new double[] {1.0, -1.0});
      assertTrue(expectedPolynomial.epsilonEquals(determinant, 1e-7));

      // From J.Pratt MCSI Problem Set 5.

      matrixA = new Matrix(new double[][]
      {
         {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}, {-1.0, 1.0, 0.0, 0.0}, {1.0, -1.0, 0.0, 0.0}
      });

      sIMinusA = PolynomialMatrix.constructSIMinusA(matrixA);
      determinant = sIMinusA.computeDeterminant();

      expectedPolynomial = new ObsoletePolynomial(new double[] {1.0, 0.0, 2.0, 0.0, 0.0});
      assertTrue(expectedPolynomial.epsilonEquals(determinant, 1e-7));
   }

	@Test
   public void testComputeDeterminantAndCofactors()
   {
      // From J.Pratt MCSI Problem Set 3.

      double[][] elementsA = new double[][]
      {
         {2.0, -2.0, 3.0}, {1.0, 1.0, 1.0}, {1.0, 3.0, -1.0}
      };
      Matrix matrixA = new Matrix(elementsA);

      PolynomialMatrix sIMinusA = PolynomialMatrix.constructSIMinusA(matrixA);
      ObsoletePolynomial determinant = sIMinusA.computeDeterminant();

      ObsoletePolynomial expectedPolynomial =
         ObsoletePolynomial.constructFromRealRoot(1.0).times(ObsoletePolynomial.constructFromRealRoot(-2.0)).times(ObsoletePolynomial.constructFromRealRoot(3.0));
      assertTrue(expectedPolynomial.epsilonEquals(determinant, 1e-7));

      ObsoletePolynomial[][] cofactors = sIMinusA.computeCofactors();


      assertTrue(cofactors[0][0].epsilonEquals(new ObsoletePolynomial(1.0, 0.0, -4.0), 1e-7));
      assertTrue(cofactors[0][1].epsilonEquals(new ObsoletePolynomial(1.0, 2.0), 1e-7));
      assertTrue(cofactors[0][2].epsilonEquals(new ObsoletePolynomial(1.0, 2.0), 1e-7));

      assertTrue(cofactors[1][0].epsilonEquals(new ObsoletePolynomial(-2.0, 7.0), 1e-7));
      assertTrue(cofactors[1][1].epsilonEquals(new ObsoletePolynomial(1.0, -1.0, -5.0), 1e-7));
      assertTrue(cofactors[1][2].epsilonEquals(new ObsoletePolynomial(3.0, -8.0), 1e-7));

      assertTrue(cofactors[2][0].epsilonEquals(new ObsoletePolynomial(3.0, -5.0), 1e-7));
      assertTrue(cofactors[2][1].epsilonEquals(new ObsoletePolynomial(1.0, 1.0), 1e-7));
      assertTrue(cofactors[2][2].epsilonEquals(new ObsoletePolynomial(1.0, -3.0, +4.0), 1e-7));

   }

}
