package us.ihmc.math.linearDynamicSysems;

import Jama.Matrix;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.math.ComplexPolynomialTools;
import us.ihmc.commons.trajectories.core.Polynomial;
import us.ihmc.commons.trajectories.interfaces.PolynomialBasics;
import us.ihmc.math.linearDynamicSystems.PolynomialMatrix;

import static org.junit.jupiter.api.Assertions.*;

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
      PolynomialBasics determinant = m0.computeDeterminant();
      assertTrue(determinant.epsilonEquals(one, 1e-7));

      polynomials = new Polynomial[][]
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

      Polynomial s = new Polynomial(false,  1.0, 0.0);
      Polynomial zero = new Polynomial(false, 0.0);
      Polynomial one = new Polynomial(false, 1.0);
      PolynomialBasics negativeOne = one.times(-1.0);

      PolynomialBasics[][] polynomials = new PolynomialBasics[][]
      {
         {s, zero, negativeOne, zero}, {zero, s, zero, negativeOne}, {one, negativeOne, s, zero}, {negativeOne, one, zero, s}
      };

      Polynomial expectedDeterminant = new Polynomial(false, 1.0, 0.0, 2.0, 0.0, 0.0);
      PolynomialMatrix m0 = new PolynomialMatrix(polynomials);
      PolynomialBasics determinant = m0.computeDeterminant();

      assertTrue(determinant.epsilonEquals(expectedDeterminant, 1e-7));
   }

	@Test
   public void testComputeDeterminantAndConstructSIMinusA()
   {
      SimpleMatrix matrixA = new SimpleMatrix(new double[][]
      {
         {1.0}
      });

      PolynomialMatrix sIMinusA = PolynomialMatrix.constructSIMinusA(matrixA);
      PolynomialBasics determinant = sIMinusA.computeDeterminant();

      Polynomial expectedPolynomial = new Polynomial(-1.0, 1.0);
      assertTrue(expectedPolynomial.epsilonEquals(determinant, 1e-7));

      // From J.Pratt MCSI Problem Set 5.

      matrixA = new SimpleMatrix(new double[][]
      {
         {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}, {-1.0, 1.0, 0.0, 0.0}, {1.0, -1.0, 0.0, 0.0}
      });

      sIMinusA = PolynomialMatrix.constructSIMinusA(matrixA);
      determinant = sIMinusA.computeDeterminant();

      expectedPolynomial = new Polynomial(false, 1.0, 0.0, 2.0, 0.0, 0.0);
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
      Matrix jama = new Matrix(elementsA);
      SimpleMatrix matrixA = new SimpleMatrix(elementsA);
      assertEquals(jama.get(0, 0), matrixA.get(0, 0));
      assertEquals(jama.get(1, 2), matrixA.get(1, 2));

      PolynomialMatrix sIMinusA = PolynomialMatrix.constructSIMinusA(matrixA);
      PolynomialBasics determinant = sIMinusA.computeDeterminant();

      PolynomialBasics expectedPolynomial =
         ComplexPolynomialTools.constructFromRealRoot(1.0).times(ComplexPolynomialTools.constructFromRealRoot(-2.0)).times(ComplexPolynomialTools.constructFromRealRoot(3.0));
      assertTrue(expectedPolynomial.epsilonEquals(determinant, 1e-7));

      PolynomialBasics[][] cofactors = sIMinusA.computeCofactors();

      assertTrue(cofactors[0][0].epsilonEquals(new Polynomial(false, 1.0, 0.0, -4.0), 1e-7));
      assertTrue(cofactors[0][1].epsilonEquals(new Polynomial(false, 1.0, 2.0), 1e-7));
      assertTrue(cofactors[0][2].epsilonEquals(new Polynomial(false, 1.0, 2.0), 1e-7));

      assertTrue(cofactors[1][0].epsilonEquals(new Polynomial(false, -2.0, 7.0), 1e-7));
      assertTrue(cofactors[1][1].epsilonEquals(new Polynomial(false, 1.0, -1.0, -5.0), 1e-7));
      assertTrue(cofactors[1][2].epsilonEquals(new Polynomial(false, 3.0, -8.0), 1e-7));

      assertTrue(cofactors[2][0].epsilonEquals(new Polynomial(false, 3.0, -5.0), 1e-7));
      assertTrue(cofactors[2][1].epsilonEquals(new Polynomial(false, 1.0, 1.0), 1e-7));
      assertTrue(cofactors[2][2].epsilonEquals(new Polynomial(false, 1.0, -3.0, +4.0), 1e-7));

   }

}
