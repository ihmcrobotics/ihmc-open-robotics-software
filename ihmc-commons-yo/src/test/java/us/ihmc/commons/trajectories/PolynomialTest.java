package us.ihmc.commons.trajectories;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.trajectories.core.Polynomial;
import us.ihmc.commons.trajectories.interfaces.PolynomialBasics;

public class PolynomialTest extends PolynomialBasicsTest
{
   @Override
   public PolynomialBasics getPolynomial(int maxNumberOfCoefficients)
   {
      return new Polynomial(maxNumberOfCoefficients);
   }

   @Test
   public void testQuinticTrajectory()
   {
      Polynomial quinticTrajectory = new Polynomial(-10.0, 10.0, true, new double[] {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

      quinticTrajectory.setQuintic(0.0, 1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

      quinticTrajectory.compute(0.0);
      Assertions.assertEquals(quinticTrajectory.getValue(), 1.0, 1e-7);
      Assertions.assertEquals(quinticTrajectory.getVelocity(), 2.0, 1e-7);
      Assertions.assertEquals(quinticTrajectory.getAcceleration(), 3.0, 1e-7);

      quinticTrajectory.compute(1.0);
      Assertions.assertEquals(quinticTrajectory.getValue(), 4.0, 1e-7);
      Assertions.assertEquals(quinticTrajectory.getVelocity(), 5.0, 1e-7);
      Assertions.assertEquals(quinticTrajectory.getAcceleration(), 6.0, 1e-7);

      quinticTrajectory.setQuintic(-1.0, 1.0, 1.0, -2.0, 3.0, -4.0, -5.0, 6.0);

      quinticTrajectory.compute(-1.0);
      Assertions.assertEquals(quinticTrajectory.getValue(), 1.0, 1e-7);
      Assertions.assertEquals(quinticTrajectory.getVelocity(), -2.0, 1e-7);
      Assertions.assertEquals(quinticTrajectory.getAcceleration(), 3.0, 1e-7);

      quinticTrajectory.compute(1.0);
      Assertions.assertEquals(quinticTrajectory.getValue(), -4.0, 1e-7);
      Assertions.assertEquals(quinticTrajectory.getVelocity(), -5.0, 1e-7);
      Assertions.assertEquals(quinticTrajectory.getAcceleration(), 6.0, 1e-7);
   }

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
   public void testSetQuintic()
   {
      Polynomial quintic = new Polynomial(false, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
      quintic.setQuintic(0.0, 1.0,    1.0, 2.0, 3.0,     4.0, 5.0, 6.0);

      quintic.compute(0.0);
      Assertions.assertEquals(quintic.getValue(), 1.0, 1e-7);
      Assertions.assertEquals(quintic.getVelocity(), 2.0, 1e-7);
      Assertions.assertEquals(quintic.getAcceleration(), 3.0, 1e-7);

      quintic.compute(1.0);
      Assertions.assertEquals(quintic.getValue(), 4.0, 1e-7);
      Assertions.assertEquals(quintic.getVelocity(), 5.0, 1e-7);
      Assertions.assertEquals(quintic.getAcceleration(), 6.0, 1e-7);

      quintic.setQuintic(-1.0, 1.0,  1.0, -2.0, 3.0,     -4.0, -5.0, 6.0);

      quintic.compute(-1.0);
      Assertions.assertEquals(quintic.getValue(), 1.0, 1e-7);
      Assertions.assertEquals(quintic.getVelocity(), -2.0, 1e-7);
      Assertions.assertEquals(quintic.getAcceleration(), 3.0, 1e-7);

      quintic.compute(1.0);
      Assertions.assertEquals(quintic.getValue(), -4.0, 1e-7);
      Assertions.assertEquals(quintic.getVelocity(), -5.0, 1e-7);
      Assertions.assertEquals(quintic.getAcceleration(), 6.0, 1e-7);
   }

   @Test
   public void testSetCubic()
   {
      Polynomial quintic = new Polynomial(1.0, 1.0, 1.0, 1.0);
      quintic.setCubic(-1.0, 1.0, 1.0, 2.0, 3.0, 4.0);

      quintic.compute(-1.0);
      Assertions.assertEquals(quintic.getValue(), 1.0, 1e-7);
      Assertions.assertEquals(quintic.getVelocity(), 2.0, 1e-7);
      quintic.compute(1.0);
      Assertions.assertEquals(quintic.getValue(), 3.0, 1e-7);
      Assertions.assertEquals(quintic.getVelocity(), 4.0, 1e-7);

      quintic.setCubic(0.0, 1.0, 3.0, 1.0, 4.0, 2.0);

      quintic.compute(0.0);
      Assertions.assertEquals(quintic.getValue(), 3.0, 1e-7);
      Assertions.assertEquals(quintic.getVelocity(), 1.0, 1e-7);
      quintic.compute(1.0);
      Assertions.assertEquals(quintic.getValue(), 4.0, 1e-7);
      Assertions.assertEquals(quintic.getVelocity(), 2.0, 1e-7);
   }

   @Test
   public void testDerivatives()
   {
      Assertions.assertEquals(1, constant5Polynomial.getNumberOfCoefficients());
      Assertions.assertEquals(constant5Polynomial.getDerivativeCoefficient(0), 0.0, 1e-15);

      Assertions.assertEquals(2, twoXPlus3Polynomial.getNumberOfCoefficients());
      Assertions.assertEquals(2.0, twoXPlus3Polynomial.getDerivativeCoefficient(0), 1e-15);

      Assertions.assertEquals(5, fourX4ThreeX3TwoX2OneX1Polynomial.getNumberOfCoefficients());
      Assertions.assertEquals(16.0, fourX4ThreeX3TwoX2OneX1Polynomial.getDerivativeCoefficient(3), 1e-15);
      Assertions.assertEquals(9.0, fourX4ThreeX3TwoX2OneX1Polynomial.getDerivativeCoefficient(2), 1e-15);
      Assertions.assertEquals(4.0, fourX4ThreeX3TwoX2OneX1Polynomial.getDerivativeCoefficient(1), 1e-15);
      Assertions.assertEquals(1.0, fourX4ThreeX3TwoX2OneX1Polynomial.getDerivativeCoefficient(0), 1e-15);
   }

   @Test
   public void testDoubleDerivatives()
   {
      Assertions.assertEquals(1, constant5Polynomial.getNumberOfCoefficients());
      Assertions.assertEquals(0.0, constant5Polynomial.getDoubleDerivativeCoefficient(0), 1e-15);

      Assertions.assertEquals(2, twoXPlus3Polynomial.getNumberOfCoefficients());
      Assertions.assertEquals(0.0, twoXPlus3Polynomial.getDoubleDerivativeCoefficient(0), 1e-15);

      Assertions.assertEquals(5, fourX4ThreeX3TwoX2OneX1Polynomial.getNumberOfCoefficients() );
      Assertions.assertEquals(fourX4ThreeX3TwoX2OneX1Polynomial.getDoubleDerivativeCoefficient(2), 48.0, 1e-15);
      Assertions.assertEquals(fourX4ThreeX3TwoX2OneX1Polynomial.getDoubleDerivativeCoefficient(1), 18.0, 1e-15);
      Assertions.assertEquals(fourX4ThreeX3TwoX2OneX1Polynomial.getDoubleDerivativeCoefficient(0), 4.0, 1e-15);
   }

   @Test
   public void testGetOrder()
   {
      Assertions.assertEquals(0, constant5Polynomial.getOrder());
      Assertions.assertEquals(1, twoXPlus3Polynomial.getOrder());
      Assertions.assertEquals(4, fourX4ThreeX3TwoX2OneX1Polynomial.getOrder());
   }

   @Test
   public void testGetCoefficients()
   {
      verifyEpsilonEquals(new double[] {5.0}, constant5Polynomial.getCoefficients(), 1e-7);
      verifyEpsilonEquals(new double[] {3.0, 2.0}, twoXPlus3Polynomial.getCoefficients(), 1e-7);
      verifyEpsilonEquals(new double[] {0.0, 1.0, 2.0, 3.0, 4.0}, fourX4ThreeX3TwoX2OneX1Polynomial.getCoefficients(), 1e-7);
   }

   @Test
   public void testImmutable()
   {
      double[] coefficients = constant5Polynomial.getCoefficients();
      coefficients[0] = 9.99;
      verifyEpsilonEquals(new double[] {5.0}, constant5Polynomial.getCoefficients(), 1e-7);
   }

   private void verifyEpsilonEquals(double[] expectedArray, double[] actualArray, double epsilon)
   {
      Assertions.assertEquals(expectedArray.length, actualArray.length);

      for (int i = 0; i < expectedArray.length; i++)
      {
         Assertions.assertEquals(expectedArray[i], actualArray[i], epsilon);
      }
   }

   @Test
   public void testTimes()
   {
      // (5) * (3 + 2x) = (15 + 10x)
      PolynomialBasics tenXPlus15Polynomial = constant5Polynomial.times(twoXPlus3Polynomial);
      verifyEpsilonEquals(new double[] {15.0, 10.0}, tenXPlus15Polynomial.getCoefficients(), 1e-7);

      // (3 + 2x) * (x + 2x^2 + 3x^3 + 4x^4) = 3x + 8x^2 + 13x^3 + 18x^4 + 8x^5
      PolynomialBasics multipliedPolynomial = twoXPlus3Polynomial.times(fourX4ThreeX3TwoX2OneX1Polynomial);
      verifyEpsilonEquals(new double[] {0.0, 3.0, 8.0, 13.0, 18.0, 8.0 }, multipliedPolynomial.getCoefficients(), 1e-7);
   }

   @Test
   public void testTimesScalar()
   {
      PolynomialBasics eighteenXPlus27Polynomial = twoXPlus3Polynomial.times(9.0);
      verifyEpsilonEquals(new double[] {27.0, 18.0}, eighteenXPlus27Polynomial.getCoefficients(), 1e-7);
   }

   @Test
   public void testPlus()
   {
      PolynomialBasics twoXPlus8Polynomial = constant5Polynomial.plus(twoXPlus3Polynomial);
      verifyEpsilonEquals(new double[] {8.0, 2.0}, twoXPlus8Polynomial.getCoefficients(), 1e-7);

      PolynomialBasics plusPolynomial = twoXPlus3Polynomial.plus(fourX4ThreeX3TwoX2OneX1Polynomial);
      verifyEpsilonEquals(new double[] {3.0, 3.0, 2.0, 3.0, 4.0}, plusPolynomial.getCoefficients(), 1e-7);

      Polynomial zero = new Polynomial(0.0);
      Polynomial zero2 = new Polynomial(0.0);

      PolynomialBasics zero3 = zero.plus(zero2);

      Assertions.assertTrue(zero3.epsilonEquals(zero, 1e-7));
   }

   @Test
   public void testEpsilonEquals()
   {
      Assertions.assertTrue(constant5Polynomial.epsilonEquals(constant5Polynomial, 1e-30));
      Assertions.assertTrue(twoXPlus3Polynomial.epsilonEquals(twoXPlus3Polynomial.plus(new Polynomial(0.0)), 1e-30));

      Assertions.assertFalse(twoXPlus3Polynomial.epsilonEquals(twoXPlus3Polynomial.plus(new Polynomial(1.0)), 1e-1));
   }


   @Test
   public void testEqualsZero()
   {
      Polynomial zeroPolynomial = new Polynomial(0.0);
      Polynomial zeroPolynomial2 = new Polynomial(0.0);
      Polynomial nonZeroPolynomial = new Polynomial(0.1, 1.0);
      Polynomial nonZeroPolynomial2 = new Polynomial(0.1, 0.0);

      Assertions.assertTrue(zeroPolynomial.equalsZero(1e-15));
      Assertions.assertTrue(zeroPolynomial2.equalsZero(1e-15));
      Assertions.assertFalse(nonZeroPolynomial.equalsZero(1e-15));
      Assertions.assertFalse(nonZeroPolynomial2.equalsZero(1e-15));
   }
}