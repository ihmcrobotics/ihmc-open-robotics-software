package us.ihmc.math;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.trajectories.core.Polynomial;
import us.ihmc.commons.trajectories.core.PolynomialMath;
import us.ihmc.commons.trajectories.interfaces.PolynomialBasics;

public class ComplexPolynomialToolsTest
{
   private Polynomial constant5Polynomial, twoXPlus3Polynomial, fourX4ThreeX3TwoX2OneX1Polynomial;

   @BeforeEach
   public void setUp()
   {
      constant5Polynomial = new Polynomial(5.0);
      twoXPlus3Polynomial = new Polynomial(false, 2.0, 3.0);
      fourX4ThreeX3TwoX2OneX1Polynomial = new Polynomial(false, 4.0, 3.0, 2.0, 1.0, 0.0);
   }

   @AfterEach
   public void tearDown()
   {
      constant5Polynomial = null;
      twoXPlus3Polynomial = null;
      fourX4ThreeX3TwoX2OneX1Polynomial = null;
   }

   @Test
   public void testConstructFromRealRoot()
   {
      double realRoot = 7.7;

      Polynomial polynomial = ComplexPolynomialTools.constructFromRealRoot(realRoot);

      Assertions.assertEquals(0.0, PolynomialMath.evaluate(polynomial, realRoot), 1e-7);
      Assertions.assertEquals(-realRoot, PolynomialMath.evaluate(polynomial, 0.0), 1e-7);
   }

   @Test
   public void testConstructFromScaleFactorAndRoots()
   {
      double scaleFactor = 3.3;
      double[] realRoots = new double[] {1.1, 2.7, 3.91};
      ComplexNumber[] complexRootPairs = new ComplexNumber[] {new ComplexNumber(10.2, 7.7), new ComplexNumber(7.9, 3.3)};

      PolynomialBasics polynomial = ComplexPolynomialTools.constructFromScaleFactorAndRoots(scaleFactor, realRoots, complexRootPairs);

      for (Double realRoot : realRoots)
      {
         Assertions.assertEquals(0.0, PolynomialMath.evaluate(polynomial, realRoot), 1e-7);
      }

      for (ComplexNumber complexRoot : complexRootPairs)
      {
         verifyEpsilonEquals(new ComplexNumber(0.0, 0.0), ComplexPolynomialTools.evaluate(polynomial, complexRoot), 1e-6);
      }
   }

   @Test
   public void testConstructFromComplexPairRoot()
   {
      verifyComplexPair(2.1, 3.3);
      verifyComplexPair(5.0, 0.0);
      verifyComplexPair(0.0, 5.0);
   }

   private void verifyComplexPair(double real, double imag)
   {
      ComplexNumber complexNumber = new ComplexNumber(real, imag);
      Polynomial polynomial = ComplexPolynomialTools.constructFromComplexPairRoot(complexNumber);

      double x = 3.7;

      double expectedValue = x * x - 2.0 * real * x + real * real + imag * imag;
      double evaluation = PolynomialMath.evaluate(polynomial, x);

      Assertions.assertEquals(expectedValue, evaluation, 1e-7);
   }

   @Test
   public void testEvaluate()
   {
      verifyEvaluations(0.0);
      verifyEvaluations(1.0);
      verifyEvaluations(7.17);

      verifyEvaluations(new ComplexNumber(0.0, 0.0));
      verifyEvaluations(new ComplexNumber(1.0, 0.0));
      verifyEvaluations(new ComplexNumber(0.0, 1.0));
      verifyEvaluations(new ComplexNumber(5.76, 3.96));
   }

   private void verifyEvaluations(double x)
   {
      double x2 = x * x;
      double x3 = x * x * x;
      double x4 = x * x * x * x;

      double epsilon = 1e-7;

      constant5Polynomial.compute(x);
      twoXPlus3Polynomial.compute(x);
      fourX4ThreeX3TwoX2OneX1Polynomial.compute(x);
      Assertions.assertEquals(5.0, constant5Polynomial.getValue(), epsilon);
      Assertions.assertEquals(2.0 * x + 3.0, twoXPlus3Polynomial.getValue(), epsilon);
      Assertions.assertEquals(4.0 * x4 + 3.0 * x3 + 2.0 * x2 + 1.0 * x + 0.0, fourX4ThreeX3TwoX2OneX1Polynomial.getValue(), epsilon);
   }

   private void verifyEvaluations(ComplexNumber x)
   {
      ComplexNumber x2 = x.times(x);
      ComplexNumber x3 = x2.times(x);
      ComplexNumber x4 = x3.times(x);

      double epsilon = 1e-7;

      verifyEpsilonEquals(new ComplexNumber(5.0, 0.0), ComplexPolynomialTools.evaluate(constant5Polynomial, x), epsilon);

      ComplexNumber twoXPlus3 = x.times(2.0).plus(new ComplexNumber(3.0, 0.0));
      verifyEpsilonEquals(twoXPlus3, ComplexPolynomialTools.evaluate(twoXPlus3Polynomial, x), epsilon);

      ComplexNumber fourX4ThreeX3TwoX2OneX1 = x4.times(4.0).plus(x3.times(3.0).plus(x2.times(2.0).plus(x.times(1.0))));
      verifyEpsilonEquals(fourX4ThreeX3TwoX2OneX1, ComplexPolynomialTools.evaluate(fourX4ThreeX3TwoX2OneX1Polynomial, x), 1e-7);
   }

   private void verifyEpsilonEquals(ComplexNumber expectedComplexNumber, ComplexNumber actualComplexNumber, double epsilon)
   {
      Assertions.assertEquals(expectedComplexNumber.real(), actualComplexNumber.real(), epsilon);
      Assertions.assertEquals(expectedComplexNumber.imaginary(), actualComplexNumber.imaginary(), epsilon);
   }
}