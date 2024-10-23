package us.ihmc.math.linearDynamicSysems;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.AngleTools;
import us.ihmc.commons.trajectories.core.Polynomial;
import us.ihmc.math.linearDynamicSystems.TransferFunction;

import static org.junit.jupiter.api.Assertions.*;

public class TransferFunctionTest
{
   private static double epsilon = 1e-7;

   @BeforeEach
   public void setUp()
   {
   }

   @AfterEach
   public void tearDown()
   {
   }

   @Test
   public void testGetNumeratorAndGetDenominatorCoefficients()
   {
      double[] numerator = new double[] {2.0, 1.0};
      double[] denominator = new double[] {4.0, 3.0};

      TransferFunction transferFunction = new TransferFunction(numerator, denominator);

      double[] expectedNumerator = new double[] {2.0 / 3.0, 1.0 / 3.0};
      double[] expectedDenominator = new double[] {4.0 / 3.0, 3.0 / 3.0};

      assertArrayEquals(expectedNumerator, transferFunction.getNumeratorCoefficients(), 1e-7);
      assertArrayEquals(expectedDenominator, transferFunction.getDenominatorCoefficients(), 1e-7);
   }

   @Test
   public void testOneOverOne()
   {
      double[] numerator = new double[] {1};
      double[] denominator = new double[] {1};

      TransferFunction transferFunction = new TransferFunction(numerator, denominator);

      double frequency = 1.0;
      double magnitude = transferFunction.getMagnitude(frequency);

      assertEquals(magnitude, 1.0, epsilon);
   }

   @Test
   public void testZero()
   {
      double[] numerator = new double[] {0};
      double[] denominator = new double[] {1};

      TransferFunction transferFunction = new TransferFunction(numerator, denominator);

      double frequency = 1.0;
      double magnitude = transferFunction.getMagnitude(frequency);
      assertEquals(magnitude, 0.0, epsilon);
   }

   @Test
   public void testOneOverS2()
   {
      double[] numerator = new double[] {1.0};
      double[] denominator = new double[] {0.0, 0.0, 1.0};

      TransferFunction transferFunction = new TransferFunction(numerator, denominator);

      assertEquals(1.0, transferFunction.getMagnitude(1.0), epsilon);
      Assertions.assertEquals(0.0, AngleTools.computeAngleDifferenceMinusPiToPi(Math.PI, transferFunction.getPhase(1.0)), 1e-7);

      assertEquals(1.0 / 25.0, transferFunction.getMagnitude(5.0), 1e-7);
      Assertions.assertEquals(0.0, AngleTools.computeAngleDifferenceMinusPiToPi(Math.PI, transferFunction.getPhase(5.0)), 1e-7);
   }

   @Test
   public void testOneOverSplusA()
   {
      double a = 3.0;
      double[] numerator = new double[] {1.0};
      double[] denominator = new double[] {a, 1.0};

      TransferFunction transferFunction = new TransferFunction(numerator, denominator);

      double omega = 1.0;
      assertEquals(1.0 / Math.sqrt(omega * omega + a * a), transferFunction.getMagnitude(omega), 1e-7);
      Assertions.assertEquals(0.0, AngleTools.computeAngleDifferenceMinusPiToPi(-Math.atan2(omega, a), transferFunction.getPhase(omega)), 1e-7);
      omega = 5.0;
      assertEquals(1.0 / Math.sqrt(omega * omega + a * a), transferFunction.getMagnitude(omega), 1e-7);
      Assertions.assertEquals(0.0, AngleTools.computeAngleDifferenceMinusPiToPi(-Math.atan2(omega, a), transferFunction.getPhase(omega)), 1e-7);
   }

   @Test
   public void testSecondOrderResponse()
   {
      double wn = 30.0;
      double zeta = 0.1;

      TransferFunction transferFunction = TransferFunction.constructSecondOrderTransferFunction(1.0, wn, zeta);

      verify(transferFunction, 0.0, 1.0, 0.0);
      verify(transferFunction, wn, 1.0 / (2.0 * zeta), -Math.PI / 2.0);
      verify(transferFunction, 100000000.0, 0.0, -Math.PI);
   }

   @Test
   public void testSecondOrderResponseArray()
   {
      double wn = 30.0;
      double zeta = 0.1;
      double[] numerator = new double[] {wn * wn};
      double[] denominator = new double[] {wn * wn, 2.0 * zeta * wn, 1.0};

      TransferFunction transferFunction = new TransferFunction(numerator, denominator);

      double[] w = new double[] {0.0, wn, 100000000.0};
      double[] expectedMagnitude = new double[] {1.0, 1.0 / (2.0 * zeta), 0.0};
      double[] expectedPhase = new double[] {0.0, -Math.PI / 2.0, -Math.PI};

      verify(transferFunction, w, expectedMagnitude, expectedPhase);
   }

   @Test
   public void testTimes()
   {
      double a1 = 1.0, a2 = 2.0, b1 = 3.0, b2 = 4.0;
      double[] numerator1 = new double[] {a1, 1.0};
      double[] numerator2 = new double[] {a2, 1.0};
      double[] denominator1 = new double[] {b1, 1.0};
      double[] denominator2 = new double[] {b2, 1.0};

      TransferFunction transferFunction1 = new TransferFunction(numerator1, denominator1);
      TransferFunction transferFunction2 = new TransferFunction(numerator2, denominator2);

      TransferFunction transferFunctionProduct = transferFunction1.times(transferFunction2);

      double[] productNumerator = transferFunctionProduct.getNumeratorCoefficients();
      double[] productDenominator = transferFunctionProduct.getDenominatorCoefficients();

      double[] expectedNumerator = new double[] {(a1 * a2), (a1 + a2), 1.0};
      double[] expectedDenominator = new double[] {(b1 * b2), (b1 + b2), 1.0};

      assertArrayEquals(expectedNumerator, productNumerator, 1e-7);
      assertArrayEquals(expectedDenominator, productDenominator, 1e-7);
   }

   @Test
   public void testPlus()
   {
      // (a1 + x) / (b1 + x) + (a2 + x) / (b2 + x) = ((a1 + x) (b2 + x) + (a2 + x) (b1 + x)) / ((b1 + x) (b2 + x))
      double a1 = 1.0, a2 = 2.0, b1 = 3.0, b2 = 4.0;
      double[] numerator1 = new double[] {a1, 1.0};
      double[] numerator2 = new double[] {a2, 1.0};
      double[] denominator1 = new double[] {b1, 1.0};
      double[] denominator2 = new double[] {b2, 1.0};

      TransferFunction transferFunction1 = new TransferFunction(numerator1, denominator1);
      TransferFunction transferFunction2 = new TransferFunction(numerator2, denominator2);

      TransferFunction transferFunctionSum = transferFunction1.plus(transferFunction2);

      double[] sumNumerator = transferFunctionSum.getNumeratorCoefficients();
      double[] sumDenominator = transferFunctionSum.getDenominatorCoefficients();

      double[] expectedNumerator = new double[] {(a1 * b2 + a2 * b1), (a1 + a2 + b1 + b2), 2.0};
      double[] expectedDenominator = new double[] {(b1 * b2), b1 + b2, 1.0};

      assertArrayEquals(expectedDenominator, sumDenominator, 1e-7);
      assertArrayEquals(expectedNumerator, sumNumerator, 1e-7);
   }

   @Test
   public void testPlusWithSameDenominator()
   {
      double a1 = 1.0, a2 = 2.0, b1 = 3.0;
      double[] numerator1 = new double[] {a1, 1.0};
      double[] numerator2 = new double[] {a2, 1.0};
      double[] denominator1 = new double[] {b1, 1.0};
      double[] denominator2 = new double[] {b1, 1.0};

      TransferFunction transferFunction1 = new TransferFunction(numerator1, denominator1);
      TransferFunction transferFunction2 = new TransferFunction(numerator2, denominator2);

      TransferFunction transferFunctionSum = transferFunction1.plus(transferFunction2);

      double[] sumNumerator = transferFunctionSum.getNumeratorCoefficients();
      double[] sumDenominator = transferFunctionSum.getDenominatorCoefficients();

      double[] expectedNumerator = new double[] {a1 + a2, 2.0};
      double[] expectedDenominator = new double[] {b1, 1.0};

      assertArrayEquals(expectedNumerator, sumNumerator, 1e-7);
      assertArrayEquals(expectedDenominator, sumDenominator, 1e-7);
   }

   private void verify(TransferFunction transferFunction, double omega, double expectedMagnitude, double expectedPhase)
   {
      assertEquals(expectedMagnitude, transferFunction.getMagnitude(omega), 1e-7);
      Assertions.assertEquals(0.0, AngleTools.computeAngleDifferenceMinusPiToPi(expectedPhase, transferFunction.getPhase(omega)), 1e-7);
   }

   private void verify(TransferFunction transferFunction, double[] omega, double[] expectedMagnitude, double[] expectedPhase)
   {
      double[] magnitude = transferFunction.getMagnitude(omega);
      double[] phase = transferFunction.getPhase(omega);

      for (int i = 0; i < omega.length; i++)
      {
         assertEquals(expectedMagnitude[i], magnitude[i], 1e-7);
         Assertions.assertEquals(0.0, AngleTools.computeAngleDifferenceMinusPiToPi(expectedPhase[i], phase[i]), 1e-7);
      }
   }

   // Fixme
   @Test
   public void testEpsilonEquals()
   {
      double a = 1.0, b = 2.0, c = 3.0, d = 4.0;
      double e = 1.0, f = 2.0, g = 3.0, h = 4.0;
      double s = 1.77;

      double[] numerator1 = new double[] {a, b, c, d};
      double[] denominator1 = new double[] {e, f, g, h};

      double[] numerator2 = new double[] {a, b, c, d};
      double[] denominator2 = new double[] {e, f, g, h};

      double[] numerator3 = new double[] {a * s, b * s, c * s, d * s};
      double[] denominator3 = new double[] {e * s, f * s, g * s, h * s};

      double[] numerator4 = new double[] {a * s, b * s, c * s, d * s};
      double[] denominator4 = new double[] {e * s, f * s + 0.1, g * s, h * s};

      double[] numerator5 = new double[] {a, b, c};
      double[] denominator5 = new double[] {e, f, g, h};

      TransferFunction transferFunction1 = new TransferFunction(numerator1, denominator1);
      TransferFunction transferFunction2 = new TransferFunction(numerator2, denominator2);
      TransferFunction transferFunction3 = new TransferFunction(numerator3, denominator3);
      TransferFunction transferFunction4 = new TransferFunction(numerator4, denominator4);
      TransferFunction transferFunction5 = new TransferFunction(numerator5, denominator5);

      assertTrue(transferFunction1.epsilonEquals(transferFunction1, 1e-7));
      assertTrue(transferFunction1.epsilonEquals(transferFunction2, 1e-7));
      assertTrue(transferFunction2.epsilonEquals(transferFunction1, 1e-7));
      assertTrue(transferFunction2.epsilonEquals(transferFunction2, 1e-7));
      assertTrue(transferFunction2.epsilonEquals(transferFunction3, 1e-7));
      assertTrue(transferFunction3.epsilonEquals(transferFunction2, 1e-7));

      assertFalse(transferFunction3.epsilonEquals(transferFunction4, 1e-7));
      assertFalse(transferFunction4.epsilonEquals(transferFunction2, 1e-7));

      assertFalse(transferFunction1.epsilonEquals(transferFunction5, 1e-7));
      assertFalse(transferFunction5.epsilonEquals(transferFunction1, 1e-7));
   }

   @Test
   public void testEqualsZero()
   {
      Polynomial numerator = new Polynomial(false, 1.0, 2.0);
      Polynomial zeroNumeratorOne = new Polynomial(0.0);
      Polynomial zeroNumeratorTwo = new Polynomial(0.0, 0.0);
      Polynomial denominator = new Polynomial(false, 1.0, 2.0);

      TransferFunction nonZeroTransferFunction = new TransferFunction(numerator, denominator);
      TransferFunction zeroTransferFunction = new TransferFunction(zeroNumeratorOne, denominator);
      TransferFunction zeroTransferFunction2 = new TransferFunction(zeroNumeratorTwo, denominator);

      assertFalse(nonZeroTransferFunction.equalsZero());
      assertTrue(zeroTransferFunction.equalsZero());
      assertTrue(zeroTransferFunction2.equalsZero());
   }
}
