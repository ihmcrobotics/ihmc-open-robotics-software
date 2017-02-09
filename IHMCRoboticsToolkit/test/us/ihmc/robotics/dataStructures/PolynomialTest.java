package us.ihmc.robotics.dataStructures;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class PolynomialTest
{
   private Polynomial constant5Polynomial, twoXPlus3Polynomial, fourX4ThreeX3TwoX2OneX1Polynomial;

   @Before
   public void setUp()
   {
      constant5Polynomial = new Polynomial(new double[] {5.0});
      twoXPlus3Polynomial = new Polynomial(new double[] {2.0, 3.0});
      fourX4ThreeX3TwoX2OneX1Polynomial = new Polynomial(new double[] {4.0, 3.0, 2.0, 1.0, 0.0});
   }

   @After
   public void tearDown()
   {
      constant5Polynomial = null;
      twoXPlus3Polynomial = null;
      fourX4ThreeX3TwoX2OneX1Polynomial = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructFromRealRoot()
   {
      double realRoot = 7.7;

      Polynomial polynomial = Polynomial.constructFromRealRoot(realRoot);

      assertEquals(0.0, polynomial.evaluate(realRoot), 1e-7);
      assertEquals(-realRoot, polynomial.evaluate(0.0), 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructFromScaleFactorAndRoots()
   {
      double scaleFactor = 3.3;
      double[] realRoots = new double[] {1.1, 2.7, 3.91};
      ComplexNumber[] complexRootPairs = new ComplexNumber[] {new ComplexNumber(10.2, 7.7), new ComplexNumber(7.9, 3.3)};

      Polynomial polynomial = Polynomial.constructFromScaleFactorAndRoots(scaleFactor, realRoots, complexRootPairs);

      for (Double realRoot : realRoots)
      {
         assertEquals(0.0, polynomial.evaluate(realRoot), 1e-7);
      }

      for (ComplexNumber complexRoot : complexRootPairs)
      {
         verifyEpsilonEquals(new ComplexNumber(0.0, 0.0), polynomial.evaluate(complexRoot), 1e-6);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructFromComplexPairRoot()
   {
      verifyComplexPair(2.1, 3.3);
      verifyComplexPair(5.0, 0.0);
      verifyComplexPair(0.0, 5.0);
   }

   private void verifyComplexPair(double real, double imag)
   {
      ComplexNumber complexNumber = new ComplexNumber(real, imag);
      Polynomial polynomial = Polynomial.constructFromComplexPairRoot(complexNumber);

      double x = 3.7;

      double expectedValue = x * x - 2.0 * real * x + real * real + imag * imag;
      double evaluation = polynomial.evaluate(x);

      assertEquals(expectedValue, evaluation, 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetQuintic()
   {
	   Polynomial quintic = new Polynomial(new double[]{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
	   quintic.setQuintic(0.0, 1.0,    1.0, 2.0, 3.0,     4.0, 5.0, 6.0);
	  
	   assertEquals(quintic.evaluate(0), 1.0, 1e-7);
	   assertEquals(quintic.evaluateDerivative(0), 2.0, 1e-7);
	   assertEquals(quintic.evaluateDoubleDerivative(0), 3.0, 1e-7);
	   
	   assertEquals(quintic.evaluate(1), 4.0, 1e-7);
	   assertEquals(quintic.evaluateDerivative(1), 5.0, 1e-7);
	   assertEquals(quintic.evaluateDoubleDerivative(1), 6.0, 1e-7);
	   
	   quintic.setQuintic(-1.0, 1.0,  1.0, -2.0, 3.0,     -4.0, -5.0, 6.0);
   
	   assertEquals(quintic.evaluate(-1),  1.0, 1e-7);
      assertEquals(quintic.evaluateDerivative(-1), -2.0, 1e-7);
      assertEquals(quintic.evaluateDoubleDerivative(-1),  3.0, 1e-7);
      
      assertEquals(quintic.evaluate(1),  -4.0, 1e-7);
      assertEquals(quintic.evaluateDerivative(1),  -5.0, 1e-7);
      assertEquals(quintic.evaluateDoubleDerivative(1),   6.0, 1e-7);	   
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetCubic()
   {
	   Polynomial quintic = new Polynomial(new double[]{1.0, 1.0, 1.0, 1.0});
	   quintic.setCubic(-1.0, 1.0, 1.0, 2.0, 3.0, 4.0);

	   assertEquals(quintic.evaluate(-1),           1.0, 1e-7);
      assertEquals(quintic.evaluateDerivative(-1), 2.0, 1e-7);
      assertEquals(quintic.evaluate(1),            3.0, 1e-7);
      assertEquals(quintic.evaluateDerivative(1),  4.0, 1e-7);

	   quintic.setCubic(0.0, 1.0, 3.0, 1.0, 4.0, 2.0);
	   
      assertEquals(quintic.evaluate(0),            3.0, 1e-7);
      assertEquals(quintic.evaluateDerivative(0),  1.0, 1e-7); 
      assertEquals(quintic.evaluate(1),            4.0, 1e-7);
      assertEquals(quintic.evaluateDerivative(1),  2.0, 1e-7);  
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDerivatives()
   {
	   double[] expectedCoeffs = constant5Polynomial.getDerivativeCoefficients();
	   assertTrue(expectedCoeffs.length == 1);
	   assertTrue(expectedCoeffs[0] == 0.0);

	   expectedCoeffs = twoXPlus3Polynomial.getDerivativeCoefficients();
	   assertTrue(expectedCoeffs.length == 1);
	   assertTrue(expectedCoeffs[0] == 2.0);
	   
	   expectedCoeffs = fourX4ThreeX3TwoX2OneX1Polynomial.getDerivativeCoefficients();
	   assertTrue(expectedCoeffs.length == 4);
	   assertTrue(expectedCoeffs[0] == 16.0);	   
	   assertTrue(expectedCoeffs[1] == 9.0);	   
	   assertTrue(expectedCoeffs[2] == 4.0);	   
	   assertTrue(expectedCoeffs[3] == 1.0);	   
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDoubleDerivatives()
   {
	   double[] expectedCoeffs = constant5Polynomial.getDoubleDerivativeCoefficients();
	   assertTrue(expectedCoeffs.length == 1);
	   assertTrue(expectedCoeffs[0] == 0.0);

	   expectedCoeffs = twoXPlus3Polynomial.getDoubleDerivativeCoefficients();
	   assertTrue(expectedCoeffs.length == 1);
	   assertTrue(expectedCoeffs[0] == 0.0);
	   
	   expectedCoeffs = fourX4ThreeX3TwoX2OneX1Polynomial.getDoubleDerivativeCoefficients();
	   assertTrue(expectedCoeffs.length == 3);
	   assertTrue(expectedCoeffs[0] == 48.0);	   
	   assertTrue(expectedCoeffs[1] == 18.0);	   
	   assertTrue(expectedCoeffs[2] == 4.0);	   
   }

   private void verifyEvaluations(double x)
   {
      double x2 = x * x;
      double x3 = x * x * x;
      double x4 = x * x * x * x;

      double epsilon = 1e-7;

      assertEquals(5.0, constant5Polynomial.evaluate(x), epsilon);
      assertEquals(2.0 * x + 3.0, twoXPlus3Polynomial.evaluate(x), epsilon);
      assertEquals(4.0 * x4 + 3.0 * x3 + 2.0 * x2 + 1.0 * x + 0.0, fourX4ThreeX3TwoX2OneX1Polynomial.evaluate(x), epsilon);
   }

   private void verifyEvaluations(ComplexNumber x)
   {
      ComplexNumber x2 = x.times(x);
      ComplexNumber x3 = x2.times(x);
      ComplexNumber x4 = x3.times(x);

      double epsilon = 1e-7;

      verifyEpsilonEquals(new ComplexNumber(5.0, 0.0), constant5Polynomial.evaluate(x), epsilon);

      ComplexNumber twoXPlus3 = x.times(2.0).plus(new ComplexNumber(3.0, 0.0));
      verifyEpsilonEquals(twoXPlus3, twoXPlus3Polynomial.evaluate(x), epsilon);

      ComplexNumber fourX4ThreeX3TwoX2OneX1 = x4.times(4.0).plus(x3.times(3.0).plus(x2.times(2.0).plus(x.times(1.0))));
      verifyEpsilonEquals(fourX4ThreeX3TwoX2OneX1, fourX4ThreeX3TwoX2OneX1Polynomial.evaluate(x), 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetOrder()
   {
      assertEquals(0, constant5Polynomial.getOrder());
      assertEquals(1, twoXPlus3Polynomial.getOrder());
      assertEquals(4, fourX4ThreeX3TwoX2OneX1Polynomial.getOrder());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetCoefficients()
   {
      verifyEpsilonEquals(new double[] {5.0}, constant5Polynomial.getCoefficients(), 1e-7);
      verifyEpsilonEquals(new double[] {2.0, 3.0}, twoXPlus3Polynomial.getCoefficients(), 1e-7);
      verifyEpsilonEquals(new double[] {4.0, 3.0, 2.0, 1.0, 0.0}, fourX4ThreeX3TwoX2OneX1Polynomial.getCoefficients(), 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testImmutable()
   {
      double[] coefficients = constant5Polynomial.getCoefficients();
      coefficients[0] = 9.99;
      verifyEpsilonEquals(new double[] {5.0}, constant5Polynomial.getCoefficients(), 1e-7);
   }

   private void verifyEpsilonEquals(double[] expectedArray, double[] actualArray, double epsilon)
   {
      assertEquals(expectedArray.length, actualArray.length);

      for (int i = 0; i < expectedArray.length; i++)
      {
         assertEquals(expectedArray[i], actualArray[i], epsilon);
      }
   }

   private void verifyEpsilonEquals(ComplexNumber expectedComplexNumber, ComplexNumber actualComplexNumber, double epsilon)
   {
      assertEquals(expectedComplexNumber.real(), actualComplexNumber.real(), epsilon);
      assertEquals(expectedComplexNumber.imag(), actualComplexNumber.imag(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTimes()
   {
      Polynomial tenXPlus15Polynomial = constant5Polynomial.times(twoXPlus3Polynomial);
      verifyEpsilonEquals(new double[] {10.0, 15.0}, tenXPlus15Polynomial.getCoefficients(), 1e-7);

      Polynomial multipliedPolynomial = twoXPlus3Polynomial.times(fourX4ThreeX3TwoX2OneX1Polynomial);
      verifyEpsilonEquals(new double[]
      {
         8.0, 18.0, 13.0, 8.0, 3.0, 0.0
      }, multipliedPolynomial.getCoefficients(), 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTimesScalar()
   {
      Polynomial eighteenXPlus27Polynomial = twoXPlus3Polynomial.times(9.0);
      verifyEpsilonEquals(new double[] {18.0, 27.0}, eighteenXPlus27Polynomial.getCoefficients(), 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPlus()
   {
      Polynomial twoXPlus8Polynomial = constant5Polynomial.plus(twoXPlus3Polynomial);
      verifyEpsilonEquals(new double[] {2.0, 8.0}, twoXPlus8Polynomial.getCoefficients(), 1e-7);

      Polynomial plusPolynomial = twoXPlus3Polynomial.plus(fourX4ThreeX3TwoX2OneX1Polynomial);
      verifyEpsilonEquals(new double[] {4.0, 3.0, 2.0, 3.0, 3.0}, plusPolynomial.getCoefficients(), 1e-7);

      Polynomial zero = new Polynomial(new double[] {0.0});
      Polynomial zero2 = new Polynomial(new double[] {0.0});

      Polynomial zero3 = zero.plus(zero2);

      assertTrue(zero3.epsilonEquals(zero, 1e-7));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testEpsilonEquals()
   {
      assertTrue(constant5Polynomial.epsilonEquals(constant5Polynomial, 1e-30));
      assertTrue(twoXPlus3Polynomial.epsilonEquals(twoXPlus3Polynomial.plus(new Polynomial(new double[] {0.0})), 1e-30));

      assertFalse(twoXPlus3Polynomial.epsilonEquals(twoXPlus3Polynomial.plus(new Polynomial(new double[] {1.0})), 1e-1));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testToString()
   {
      // System.out.println(constant5Polynomial);
      // System.out.println(twoXPlus3Polynomial);
      // System.out.println(fourX4ThreeX3TwoX2OneX1);

      assertEquals("5.0", constant5Polynomial.toString());
      assertEquals("2.0 * x + 3.0", twoXPlus3Polynomial.toString());
      assertEquals("4.0 * x^4 + 3.0 * x^3 + 2.0 * x^2 + 1.0 * x + 0.0", fourX4ThreeX3TwoX2OneX1Polynomial.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testEqualsZero()
   {
      Polynomial zeroPolynomial = new Polynomial(new double[] {0.0});
      Polynomial zeroPolynomial2 = new Polynomial(new double[] {0.0, 0.0});
      Polynomial nonZeroPolynomial = new Polynomial(new double[] {1.0, 0.1});
      Polynomial nonZeroPolynomial2 = new Polynomial(new double[] {0.0, 0.1});

      assertTrue(zeroPolynomial.equalsZero());
      assertTrue(zeroPolynomial2.equalsZero());
      assertFalse(nonZeroPolynomial.equalsZero());
      assertFalse(nonZeroPolynomial2.equalsZero());
   }

}
