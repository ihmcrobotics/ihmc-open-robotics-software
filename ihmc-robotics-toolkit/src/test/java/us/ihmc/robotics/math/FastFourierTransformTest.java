package us.ihmc.robotics.math;

import org.junit.Test;
import static org.junit.Assert.*;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;

public class FastFourierTransformTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 30000)
   public void testBitReverse()
   {
      FastFourierTransform fft = FastFourierTransform.getFourierTransformer();
      assertTrue (fft.bitReverse(0, 3) == 0);
      assertTrue (fft.bitReverse(1, 3) == 4);
      assertTrue (fft.bitReverse(2, 3) == 2);
      assertTrue (fft.bitReverse(3, 3) == 6);
      assertTrue (fft.bitReverse(4, 3) == 1);
      assertTrue (fft.bitReverse(5, 3) == 5);
      assertTrue (fft.bitReverse(6, 3) == 3);
      assertTrue (fft.bitReverse(7, 3) == 7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 30000)
   public void testFourierTransform()
   {
      double[] testArray = {1, 1, 1, 1, 1};
      FastFourierTransform fft = new FastFourierTransform(8);
      fft.setCoefficients(testArray);
      ComplexNumber[] result = fft.getForwardTransform();

      fft.setCoefficients(result);
      result = fft.getInverseTransform();
      for (int i = 0; i < 5; i++)
         assertTrue (MathTools.epsilonCompare(result[i].real(), 1, Epsilons.ONE_BILLIONTH));
      for (int i = 5; i < result.length; i++)
         assertTrue (MathTools.epsilonCompare(result[i].real(), 0, Epsilons.ONE_BILLIONTH));

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 30000)
   public void testFourierTransform2()
   {
      double[] testArray1 = {1, 1};
      double[] testArray2 = {1, 1, 1};
      FastFourierTransform fft = new FastFourierTransform(4);

      fft.setCoefficients(testArray1);
      ComplexNumber[] result1 = fft.getForwardTransform();
      
      ComplexNumber[] saveVals = new ComplexNumber[4];
      for(int i = 0; i < 4; i++)
         saveVals[i] = new ComplexNumber(result1[i]);
      
      fft.setCoefficients(testArray2);
      ComplexNumber[] result2 = fft.getForwardTransform();

      for (int i = 0; i < result1.length; i++)
         saveVals[i].timesAndStore(result2[i]);

      fft.setCoefficients(saveVals);
      result1 = fft.getInverseTransform();
      
      assertTrue (MathTools.epsilonCompare(result1[0].real(), 1, Epsilons.ONE_BILLIONTH));
      assertTrue (MathTools.epsilonCompare(result1[1].real(), 2, Epsilons.ONE_BILLIONTH));
      assertTrue (MathTools.epsilonCompare(result1[2].real(), 2, Epsilons.ONE_BILLIONTH));
      assertTrue (MathTools.epsilonCompare(result1[3].real(), 1, Epsilons.ONE_BILLIONTH));
   }

}
