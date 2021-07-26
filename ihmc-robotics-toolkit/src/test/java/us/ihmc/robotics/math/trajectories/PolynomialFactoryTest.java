package us.ihmc.robotics.math.trajectories;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.core.PolynomialFactory;
import us.ihmc.robotics.math.trajectories.core.PolynomialTools;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;

import static us.ihmc.robotics.Assert.assertEquals;

public class PolynomialFactoryTest
{
   private Polynomial constant5Polynomial, twoXPlus3Polynomial, fourX4ThreeX3TwoX2OneX1Polynomial;

   @BeforeEach
   public void setUp()
   {
      constant5Polynomial = new Polynomial(new double[] {5.0}, false);
      twoXPlus3Polynomial = new Polynomial(new double[] {2.0, 3.0}, false);
      fourX4ThreeX3TwoX2OneX1Polynomial = new Polynomial(new double[] {4.0, 3.0, 2.0, 1.0, 0.0}, false);
   }

   @AfterEach
   public void tearDown()
   {
      constant5Polynomial = null;
      twoXPlus3Polynomial = null;
      fourX4ThreeX3TwoX2OneX1Polynomial = null;
   }

   @Test
   public void testTimes()
   {
      PolynomialBasics tenXPlus15Polynomial = PolynomialFactory.times(constant5Polynomial, twoXPlus3Polynomial);
      verifyEpsilonEquals(new double[] {15.0, 10.0}, tenXPlus15Polynomial.getCoefficients(), 1e-7);

      PolynomialBasics multipliedPolynomial = PolynomialFactory.times(twoXPlus3Polynomial, fourX4ThreeX3TwoX2OneX1Polynomial);
      verifyEpsilonEquals(new double[]
                                {
                                      0.0, 3.0, 8.0, 13.0, 18.0, 8.0
                                }, multipliedPolynomial.getCoefficients(), 1e-7);
   }

   @Test
   public void testTimesScalar()
   {
      PolynomialBasics eighteenXPlus27Polynomial = PolynomialFactory.copyAndScale(9.0, twoXPlus3Polynomial);
      verifyEpsilonEquals(new double[] {27.0, 18.0}, eighteenXPlus27Polynomial.getCoefficients(), 1e-7);
   }

   @Test
   public void testConstructFromScaleFactorAndRoots()
   {
      double scaleFactor = 3.3;
      double[] realRoots = new double[] {1.1, 2.7, 3.91};
      ComplexNumber[] complexRootPairs = new ComplexNumber[] {new ComplexNumber(10.2, 7.7), new ComplexNumber(7.9, 3.3)};

      PolynomialBasics polynomial = PolynomialFactory.constructFromScaleFactorAndRoots(scaleFactor, realRoots, complexRootPairs);

      for (Double realRoot : realRoots)
      {
         polynomial.compute(realRoot);

         assertEquals(0.0, polynomial.getValue(), 1e-7);
      }

      for (ComplexNumber complexRoot : complexRootPairs)
      {
         verifyEpsilonEquals(new ComplexNumber(0.0, 0.0), PolynomialTools.evaluate(polynomial, complexRoot), 1e-6);
      }
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

}
