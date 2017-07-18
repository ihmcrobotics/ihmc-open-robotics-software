package us.ihmc.commonWalkingControlModules.angularMomentumTrajectory;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.ComplexNumber;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.FastFourierTransform;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.MathTools;

public class FastFourierTransformTest
{
   @Test
   public void testBitReverse()
   {
      FastFourierTransform fft = FastFourierTransform.getFourierTransformer();
      assert (fft.bitReverse(0, 3) == 0);
      assert (fft.bitReverse(1, 3) == 4);
      assert (fft.bitReverse(2, 3) == 2);
      assert (fft.bitReverse(3, 3) == 6);
      assert (fft.bitReverse(4, 3) == 1);
      assert (fft.bitReverse(5, 3) == 5);
      assert (fft.bitReverse(6, 3) == 3);
      assert (fft.bitReverse(7, 3) == 7);
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
         assert (MathTools.epsilonCompare(result[i].getRealPart(), 1, Epsilons.ONE_BILLIONTH));
      for (int i = 5; i < result.length; i++)
         assert (MathTools.epsilonCompare(result[i].getRealPart(), 0, Epsilons.ONE_BILLIONTH));

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
      for(int i = 0; i < 4; i++)
         saveVals[i] = new ComplexNumber(result1[i]);
      
      fft.setCoefficients(testArray2);
      ComplexNumber[] result2 = fft.getForwardTransform();

      for (int i = 0; i < result1.length; i++)
         saveVals[i].multiply(result2[i]);

      fft.setCoefficients(saveVals);
      result1 = fft.getInverseTransform();
      
      assert (MathTools.epsilonCompare(result1[0].getRealPart(), 1, Epsilons.ONE_BILLIONTH));
      assert (MathTools.epsilonCompare(result1[1].getRealPart(), 2, Epsilons.ONE_BILLIONTH));
      assert (MathTools.epsilonCompare(result1[2].getRealPart(), 2, Epsilons.ONE_BILLIONTH));
      assert (MathTools.epsilonCompare(result1[3].getRealPart(), 1, Epsilons.ONE_BILLIONTH));
   }

}
