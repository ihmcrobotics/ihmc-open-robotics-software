package us.ihmc.math;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;

import static org.junit.jupiter.api.Assertions.*;

public class FastFourierTransformTest
{
   @Test
   public void testBitReverse()
   {
      assertEquals(FastFourierTransform.bitReverse(0, 3), 0);
      assertEquals(FastFourierTransform.bitReverse(1, 3), 4);
      assertEquals(FastFourierTransform.bitReverse(2, 3), 2);
      assertEquals(FastFourierTransform.bitReverse(3, 3), 6);
      assertEquals(FastFourierTransform.bitReverse(4, 3), 1);
      assertEquals(FastFourierTransform.bitReverse(5, 3), 5);
      assertEquals(FastFourierTransform.bitReverse(6, 3), 3);
      assertEquals(FastFourierTransform.bitReverse(7, 3), 7);
   }

   @Test
   public void testFourierTransform()
   {
      double[] testArray = {1, 1, 1, 1, 1};
      FastFourierTransform fft = new FastFourierTransform(8);
      fft.setCoefficients(testArray);
      ComplexNumber[] result = fft.getForwardTransform();

      fft.setCoefficients(result);
      result = fft.getInverseTransform();
      for (int i = 0; i < 5; i++)
         assertEquals(result[i].real(), 1, Epsilons.ONE_BILLIONTH);
      for (int i = 5; i < result.length; i++)
         assertEquals(result[i].real(), 0, Epsilons.ONE_BILLIONTH);
   }

   @Test
   public void testFourierTransform2()
   {
      double[] testArray1 = {1, 1};
      double[] testArray2 = {1, 1, 1};
      FastFourierTransform fft = new FastFourierTransform(4);

      fft.setCoefficients(testArray1);
      ComplexNumber[] result1 = fft.getForwardTransform();

      ComplexNumber[] saveVals = new ComplexNumber[4];
      for (int i = 0; i < 4; i++)
         saveVals[i] = new ComplexNumber(result1[i]);

      fft.setCoefficients(testArray2);
      ComplexNumber[] result2 = fft.getForwardTransform();

      for (int i = 0; i < result1.length; i++)
         saveVals[i].timesEquals(result2[i]);

      fft.setCoefficients(saveVals);
      result1 = fft.getInverseTransform();

      assertTrue(MathTools.epsilonCompare(result1[0].real(), 1, Epsilons.ONE_BILLIONTH));
      assertTrue(MathTools.epsilonCompare(result1[1].real(), 2, Epsilons.ONE_BILLIONTH));
      assertTrue(MathTools.epsilonCompare(result1[2].real(), 2, Epsilons.ONE_BILLIONTH));
      assertTrue(MathTools.epsilonCompare(result1[3].real(), 1, Epsilons.ONE_BILLIONTH));
   }
}
