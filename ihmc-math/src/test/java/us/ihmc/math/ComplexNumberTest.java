package us.ihmc.math;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;

import static org.junit.jupiter.api.Assertions.*;

public class ComplexNumberTest
{

	@Test
   public void testCommonUsage()
   {
      double real = 3.0;
      double imag = 4.0;

      ComplexNumber complexNumber = new ComplexNumber(real, imag);

      double epsilon = 1e-14;
      assertEquals(real, complexNumber.real(), epsilon);
      assertEquals(imag, complexNumber.imag(), epsilon);
      
      assertEquals(real * real + imag * imag, complexNumber.magnitudeSquared(), epsilon);
      assertEquals(Math.sqrt(real * real + imag * imag), complexNumber.magnitude(), epsilon);
   }

	@Test
   public void testZero()
   {
      ComplexNumber complexNumber = new ComplexNumber(0.0, 0.0);
      
      assertTrue(0.0 == complexNumber.magnitude());
      assertTrue(0.0 == complexNumber.magnitudeSquared());
   }

	@Test
   public void testALotOfRandomComplexNumbers()
   {
      Random random = new Random(1776L);
      
      int numberOfTests = 10000;
      double epsilon = 1e-9;
      
      for (int i=0; i<numberOfTests; i++)
      {
         ComplexNumber complexNumber = new ComplexNumber((1.0 - 2.0 * random.nextDouble()) * 1000.0, (1.0 - 2.0 * random.nextDouble()) * 1000.0);
         
         assertTrue(complexNumber.epsilonEquals(complexNumber.changeSign().changeSign(), epsilon));
         assertTrue(complexNumber.epsilonEquals(complexNumber.conj().conj(), epsilon));
         
         assertTrue(complexNumber.epsilonEquals(new ComplexNumber(complexNumber), 1e-18));
      }
   }

	@Test
   public void testEpsilonEquals()
   {
      double epsilon = 0.1;
      
      ComplexNumber complexNumber = new ComplexNumber(1.9997, 88.245);

      ComplexNumber aLittleDifferent = complexNumber.plus(0.5 * epsilon);
      assertTrue(complexNumber.epsilonEquals(aLittleDifferent, epsilon));
      
      aLittleDifferent = complexNumber.plus(2.0 * epsilon);
      assertFalse(complexNumber.epsilonEquals(aLittleDifferent, epsilon));
      
      aLittleDifferent = complexNumber.plus(new ComplexNumber(0.0, 0.5 * epsilon));
      assertTrue(complexNumber.epsilonEquals(aLittleDifferent, epsilon));
      
      aLittleDifferent = complexNumber.plus(new ComplexNumber(0.0, 2.0 * epsilon));
      assertFalse(complexNumber.epsilonEquals(aLittleDifferent, epsilon));
      
      complexNumber = new ComplexNumber(99.2, 0.0);
      assertTrue(complexNumber.epsilonEquals(complexNumber.real() + epsilon / 2.0, epsilon));
      assertFalse(complexNumber.epsilonEquals(complexNumber.real() + 2.0 * epsilon, epsilon));
      
      complexNumber = new ComplexNumber(104.5, epsilon/2.0);
      assertTrue(complexNumber.epsilonEquals(complexNumber.real() + epsilon / 2.0, epsilon));
      assertFalse(complexNumber.epsilonEquals(complexNumber.real() + 2.0 * epsilon, epsilon));
      
      complexNumber = new ComplexNumber(104.5, epsilon*2.0);
      assertFalse(complexNumber.epsilonEquals(complexNumber.real() + epsilon / 2.0, epsilon));
      assertFalse(complexNumber.epsilonEquals(complexNumber.real() + 2.0 * epsilon, epsilon));
   }

	@Test
   public void regressionTest()
   {
      ComplexNumber complexNumber = new ComplexNumber(5.5, 7.93);
      
      ComplexNumber changeSign = complexNumber.changeSign();
      ComplexNumber conj = complexNumber.conj();
      ComplexNumber cos = complexNumber.cos();
      ComplexNumber cosh = complexNumber.cosh();
      ComplexNumber exp = complexNumber.exp();
      ComplexNumber log = complexNumber.log();
      ComplexNumber sin = complexNumber.sin();
      ComplexNumber sinh = complexNumber.sinh();
      ComplexNumber sqrt = complexNumber.sqrt();
      ComplexNumber tan = complexNumber.tan();
     
      double angle = complexNumber.angle();
      double magnitude = complexNumber.magnitude();
      double magnitudeSquared = complexNumber.magnitudeSquared();
      
      double epsilon = 1e-10;
      assertEquals(0.9643997071734942, angle, epsilon);
      assertEquals(9.650642465659994, magnitude, epsilon);
      assertEquals(93.13489999999999, magnitudeSquared, epsilon);
      
      assertTrue(complexNumber.epsilonEquals(new ComplexNumber(5.5, 7.93), epsilon));
      assertTrue(changeSign.epsilonEquals(new ComplexNumber(-5.5, -7.93), epsilon));
      assertTrue(conj.epsilonEquals(new ComplexNumber(5.5, -7.93), epsilon));
      assertTrue(cos.epsilonEquals(new ComplexNumber(984.8480105929419, 980.4987193570814), epsilon));
      assertTrue(cosh.epsilonEquals(new ComplexNumber(-9.29174056024931, 121.99059290783434), epsilon));
      
      assertTrue(exp.epsilonEquals(new ComplexNumber(-18.583170749941022, 243.98526078449154), epsilon));
      assertTrue(log.epsilonEquals(new ComplexNumber(2.267024489887092, 0.9643997071734942), epsilon));
      assertTrue(sin.epsilonEquals(new ComplexNumber(-980.4989732009044, 984.847755623186), epsilon));
      assertTrue(sinh.epsilonEquals(new ComplexNumber(-9.291430189691711, 121.9946678766572), epsilon));
      assertTrue(sqrt.epsilonEquals(new ComplexNumber(2.752330146045346, 1.4405975263167698), epsilon));
      assertTrue(tan.epsilonEquals(new ComplexNumber(-2.5888999008384745E-7, 0.9999999988541863), epsilon));
   }

	@Test
   public void regessionTestTwo()
   {
      ComplexNumber complexNumberOne = new ComplexNumber(88.34, -17.67);
      ComplexNumber complexNumberTwo = new ComplexNumber(-16.33, 22.1234);

      ComplexNumber plus = complexNumberOne.plus(complexNumberTwo);
      ComplexNumber times = complexNumberOne.times(complexNumberTwo);
      ComplexNumber minus = complexNumberOne.minus(complexNumberTwo);
    
      double epsilon = 1e-10;
      assertTrue(plus.epsilonEquals(new ComplexNumber(72.01, 4.4533999999999985), epsilon));
      assertTrue(times.epsilonEquals(new ComplexNumber(-1051.6717219999998, 2242.932256), epsilon));
      assertTrue(minus.epsilonEquals(new ComplexNumber(104.67, -39.793400000000005), epsilon));

   }

	@Test
   public void regessionTestThree()
   {
      ComplexNumber complexNumber = new ComplexNumber(76.45, 92.345);

      ComplexNumber plus = complexNumber.plus(7.934);
      ComplexNumber times = complexNumber.times(18.356);
      ComplexNumber minus = complexNumber.minus(72.668);
    
      double epsilon = 1e-10;
      assertTrue(plus.epsilonEquals(new ComplexNumber(84.384, 92.345), epsilon));
      assertTrue(times.epsilonEquals(new ComplexNumber(1403.3162000000002, 1695.08482), epsilon));
      assertTrue(minus.epsilonEquals(new ComplexNumber(3.7819999999999965, 92.345), epsilon));
   }

   @Test
   public void testToString()
   {
      ComplexNumber complexNumber = new ComplexNumber(1.0, 2.0);
      assertEquals("1.0 + 2.0i", complexNumber.toString());
      
      complexNumber = new ComplexNumber(11.0, -12.0);
      assertEquals("11.0 - 12.0i", complexNumber.toString());
      
      complexNumber = new ComplexNumber(0.0, 5.0);
      assertEquals("5.0i", complexNumber.toString());
      
      complexNumber = new ComplexNumber(100.0, 0.0);
      assertEquals("100.0", complexNumber.toString());
      
      complexNumber = new ComplexNumber(0.0, -99.0);
      assertEquals("-99.0i", complexNumber.toString());
      
      complexNumber = new ComplexNumber(Double.NaN, -99.0);
      assertEquals("NaN - 99.0i", complexNumber.toString());
      
      complexNumber = new ComplexNumber(3.3, Double.NaN);
      assertEquals("3.3 + i*NaN", complexNumber.toString());
   }


   @Test
   public void testInitialization()
   {
      ComplexNumber c1 = new ComplexNumber();
      assert (c1.real() == 0.0);
      assert (c1.imag() == 0.0);
      c1 = new ComplexNumber(1, 1);
      assert (c1.real() == 1.0);
      assert (c1.imag() == 1.0);
      c1.set(2.12, 3.14159);
      assert (c1.real() == 2.12);
      assert (c1.imag() == 3.14159);
      c1.setFromEuler(2, Math.toRadians(90));
      assert (Math.abs(c1.real()) < Epsilons.ONE_BILLIONTH);
      assert (c1.imag() == 2);
   }

   @Test
   public void testMath()
   {
      ComplexNumber c1 = new ComplexNumber(2.4, 3.1);
      ComplexNumber c2 = new ComplexNumber(1.2, 4.5);
      ComplexNumber c3 = new ComplexNumber();
      c3.plusAndStore(c1, c2);
      assert (c3.real() == c1.real() + c2.real());
      assert (c3.imag() == c1.imag() + c2.imag());
      c3.timesAndStore(c1, c2);
      assert (c3.real() == 2.4 * 1.2 - 4.5 * 3.1);
      assert (c3.imag() == 2.4 * 4.5 + 1.2 * 3.1);
      c3.setToPurelyReal(1);
      assert (c3.magnitude() == 1);
      assert (c3.angle() == 0);
      c3.getRoots(c1, c2);
      assert (c1.real() == 1);
      assert (c1.imag() == 0);
      assert (c2.real() == -1);
      assert (Math.abs(c2.imag()) < Epsilons.ONE_BILLIONTH);
      c2.getRoots(c1, c3);
      assert (Math.abs(c1.real() - 0) < Epsilons.ONE_BILLIONTH);
      assert (Math.abs(c1.imag() - 1) < Epsilons.ONE_BILLIONTH);
      assert (Math.abs(c3.real() - 0) < Epsilons.ONE_BILLIONTH);
      assert (Math.abs(c3.imag() + 1) < Epsilons.ONE_BILLIONTH);
   }

   @Test
   public void tesNthRoots()
   {
      ComplexNumber c1 = new ComplexNumber(1.0, 0.0);
      List<ComplexNumber> roots = new ArrayList<>(8);
      for(int i = 0; i < 8; i++)
         roots.add(new ComplexNumber());
      c1.getRoots(roots, 8);
      for(int i = 0; i < roots.size(); i++)
      {
         assert (MathTools.epsilonCompare(roots.get(i).real(), Math.cos(i*2.0*Math.PI/8.0), Epsilons.ONE_BILLIONTH));
         assert (MathTools.epsilonCompare(roots.get(i).imag(), Math.sin(i*2.0*Math.PI/8.0), Epsilons.ONE_BILLIONTH));
      }
   }

}
