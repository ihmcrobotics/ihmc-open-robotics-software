package us.ihmc.commons.trajectories;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.trajectories.core.PolynomialTools;

import static org.junit.jupiter.api.Assertions.*;

public class PolynomialToolsTest
{
   private static double EPSILON = 1e-6;

   @Test
   public void testDerivativeCoefficients()
   {
      //cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3

      int order3Exponent1Func = PolynomialTools.getDerivativeCoefficient(3, 1);
      int order3Exponent1Hand = 0;
      assertEquals(order3Exponent1Func, order3Exponent1Hand, EPSILON);

      int order6Exponent7Func = PolynomialTools.getDerivativeCoefficient(6, 7);
      int order6Exponent7Hand = 5040;
      assertEquals(order6Exponent7Func, order6Exponent7Hand, EPSILON);

      int order0Exponent5Func = PolynomialTools.getDerivativeCoefficient(0, 5);
      int order0Exponent5Hand = 1;
      assertEquals(order0Exponent5Func, order0Exponent5Hand, EPSILON);

      int order3Exponent4Func = PolynomialTools.getDerivativeCoefficient(3, 4);
      int order3Exponent4Hand = 24;
      assertEquals(order3Exponent4Func, order3Exponent4Hand, EPSILON);

      int order5Exponent2Func = PolynomialTools.getDerivativeCoefficient(5, 2);
      int order5Exponent2Hand = 0;
      assertEquals(order5Exponent2Func, order5Exponent2Hand, EPSILON);

      int order1Exponent5Func = PolynomialTools.getDerivativeCoefficient(1, 5);
      int order1Exponent5Hand = 5;
      assertEquals(order1Exponent5Func, order1Exponent5Hand, EPSILON);

      int order11Exponent1Func = PolynomialTools.getDerivativeCoefficient(11, 1);
      int order11Exponent1Hand = 0;
      assertEquals(order11Exponent1Func, order11Exponent1Hand, EPSILON);

      int order13Exponent8Func = PolynomialTools.getDerivativeCoefficient(13, 8);
      int order13Exponent8Hand = 0;
      assertEquals(order13Exponent8Func, order13Exponent8Hand, EPSILON);
   }
}
