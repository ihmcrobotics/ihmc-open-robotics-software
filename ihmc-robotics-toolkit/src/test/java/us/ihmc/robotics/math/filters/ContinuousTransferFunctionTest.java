package us.ihmc.robotics.math.filters;

import org.apache.commons.lang3.ArrayUtils;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ContinuousTransferFunctionTest
{

   @Test
   void testNumeratorAndDenominatorSetCorrectly()
   {
      String name = "tf";
      double k = 5;
      double[] numerator = new double[] {1.0, 2.0, 3.0, 4.0, 5.0};
      double[] denominator = new double[] {5.0, 4.0, 3.0, 2.0, 1.0};

      double[] numAndDen = ArrayUtils.addAll(numerator, denominator);

      ContinuousTransferFunction tf = new ContinuousTransferFunction(k, numerator, denominator);

      double[] numAndDenOutput = ArrayUtils.addAll(tf.getNumerator(), tf.getDenominator());

      assertArrayEquals(numAndDen, numAndDenOutput);
   }

   @Test
   void testNumeratorGainSetCorrectly()
   {
      String name = "tf";
      double k = 5;
      double[] numerator = new double[] {1.0, 2.0, 3.0, 4.0, 5.0};
      double[] denominator = new double[] {5.0, 4.0, 3.0, 2.0, 1.0};

      ContinuousTransferFunction tf = new ContinuousTransferFunction(k, numerator, denominator);

      assertEquals(k, tf.getGain());
   }

   @Test
   void testCascadingTransferFunctionsNumAndDen()
   {
      String name1 = "tf1";
      double k1 = 5;
      double[] numerator1 = new double[] {1.0, 2.0, 3.0, 5.0};
      double[] denominator1 = new double[] {5.0, 4.0, 3.0, 2.0, 1.0};
      ContinuousTransferFunction tf1 = new ContinuousTransferFunction(name1, k1, numerator1, denominator1);

      String name2 = "tf2";
      double k2 = 9;
      double[] numerator2 = new double[] {10.0, 5.0, 5.0};
      double[] denominator2 = new double[] {18.0, 0.1, 1.8, 49.0, 0.0};
      ContinuousTransferFunction tf2 = new ContinuousTransferFunction(name2, k2, numerator2, denominator2);

      String name3 = "tf3";
      double k3 = 1;
      double[] numerator3 = new double[] {9.0, 0.0, 0.0, 45.0, 1.9};
      double[] denominator3 = new double[] {5.0, 4.0, 3.0, 6.0, 10.0, 1.8, 17.0, 9.0, 4.0};
      ContinuousTransferFunction tf3 = new ContinuousTransferFunction(name3, k3, numerator3, denominator3);

      String name4 = "Combine_TF";

      // Combine Transfer Functions
      ContinuousTransferFunction tf_total = new ContinuousTransferFunction(name4, new ContinuousTransferFunction[] {tf1, tf2, tf3});

      // Combine Numerator and Denominator Arrays.
      double[] numAndDenOutput = ArrayUtils.addAll(tf_total.getNumerator(), tf_total.getDenominator());

      // Set Matlab's Output
      double delta = 0.00000000001;
      double[] matlabOutputNum = new double[] {90,
                                               225,
                                               405,
                                               1125,
                                               1504,
                                               2.297500000000000e+03,
                                               3.460500000000000e+03,
                                               1.942500000000000e+03,
                                               1201,
                                               47.500000000000000};
      double[] matlabOutputDen = new double[] {450,
                                               7.225000000000000e+02,
                                               877,
                                               2.453600000000000e+03,
                                               3.777200000000000e+03,
                                               3.764800000000000e+03,
                                               5.786100000000000e+03,
                                               7.455520000000000e+03,
                                               6.205200000000000e+03,
                                               8.413180000000000e+03,
                                               8.146559999999999e+03,
                                               6.361940000000000e+03,
                                               4.019500000000000e+03,
                                               2334,
                                               8.402000000000000e+02,
                                               196,
                                               0};
      double[] matlabNumAndDenOutput = ArrayUtils.addAll(matlabOutputNum, matlabOutputDen);

      // Compare Matlab and Method's outputs.
      assertArrayEquals(matlabNumAndDenOutput, numAndDenOutput, delta);
   }

   @Test
   void testCascadingTransferFunctionsGain()
   {
      double k1 = 5;
      double[] numerator1 = new double[] {1.0, 2.0, 3.0, 5.0};
      double[] denominator1 = new double[] {5.0, 4.0, 3.0, 2.0, 1.0};
      ContinuousTransferFunction tf1 = new ContinuousTransferFunction(k1, numerator1, denominator1);

      double k2 = 9;
      double[] numerator2 = new double[] {10.0, 5.0, 5.0};
      double[] denominator2 = new double[] {18.0, 0.1, 1.8, 49.0, 0.0};
      ContinuousTransferFunction tf2 = new ContinuousTransferFunction(k2, numerator2, denominator2);

      double k3 = 1;
      double[] numerator3 = new double[] {9.0, 0.0, 0.0, 45.0, 1.9};
      double[] denominator3 = new double[] {5.0, 4.0, 3.0, 6.0, 10.0, 1.8, 17.0, 9.0, 4.0};
      ContinuousTransferFunction tf3 = new ContinuousTransferFunction(k3, numerator3, denominator3);

      // Combine Transfer Functions
      ContinuousTransferFunction tf_total = new ContinuousTransferFunction(new ContinuousTransferFunction[] {tf1, tf2, tf3});

      // Compare.
      assertEquals(k1 * k2 * k3, tf_total.getGain());
   }

   @Test
   void testCascadeButterworthAndNotchFilter()
   {
      String name_butter = "Second Order Low-Pass Butterworth Filter";
      double wc = 2 * Math.PI * 10.0;
      ContinuousTransferFunction tf_butter = new ContinuousTransferFunction(name_butter,
                                                                            1.0,
                                                                            new double[] {wc * wc},
                                                                            new double[] {1.0, Math.sqrt(2) * wc, wc * wc});

      String name_notch = "Second Order Notch Filter";
      double wn = 60 * 2 * Math.PI;
      double Q = 5.0;

      ContinuousTransferFunction tf_notch = new ContinuousTransferFunction(name_notch,
                                                                           1.0,
                                                                           new double[] {1.0, 0.0, wn * wn},
                                                                           new double[] {1.0, wn / Q, wn * wn});

      String name_combined = "Cascaded Butterworth and Notch Filter";

      // Combine Transfer Functions
      ContinuousTransferFunction tf_total = new ContinuousTransferFunction(name_combined, new ContinuousTransferFunction[] {tf_butter, tf_notch});

      // Combine Numerator and Denominator Arrays.
      double[] numAndDenOutput = ArrayUtils.addAll(tf_total.getNumerator(), tf_total.getDenominator());

      // Set Matlab's Output
      double delta = 0.00001;
      double[] matlabOutputNum = new double[] {3.947841760435743e+03, 0, 5.610763643558538e+08};
      double[] matlabOutputDen = new double[] {1, 1.642558824493224e+02, 1.527698547677758e+05, 1.292631539212299e+07, 5.610763643558538e+08};
      double[] matlabNumAndDenOutput = ArrayUtils.addAll(matlabOutputNum, matlabOutputDen);

      // Compare Matlab and Method's outputs.
      assertArrayEquals(matlabNumAndDenOutput, numAndDenOutput, delta);
   }

   @Test
   void testCascadeOrderShouldntMatter()
   {
      String name_butter = "Second Order Low-Pass Butterworth Filter";
      double wc = 2 * Math.PI * 10.0;
      ContinuousTransferFunction tf_butter = new ContinuousTransferFunction(name_butter,
                                                                            1.0,
                                                                            new double[] {wc * wc},
                                                                            new double[] {1.0, Math.sqrt(2) * wc, wc * wc});

      String name_notch = "Second Order Notch Filter";
      double wn = 60 * 2 * Math.PI;
      double Q = 5.0;

      ContinuousTransferFunction tf_notch = new ContinuousTransferFunction(name_notch,
                                                                           1.0,
                                                                           new double[] {1.0, 0.0, wn * wn},
                                                                           new double[] {1.0, wn / Q, wn * wn});

      String name_combined = "Cascaded Butterworth and Notch Filter";

      // Combine Transfer Functions
      ContinuousTransferFunction tf_total1 = new ContinuousTransferFunction(name_combined, new ContinuousTransferFunction[] {tf_butter, tf_notch});
      ContinuousTransferFunction tf_total2 = new ContinuousTransferFunction(name_combined, new ContinuousTransferFunction[] {tf_notch, tf_butter});

      // Combine Numerator and Denominator Arrays.
      double[] numAndDenOutput1 = ArrayUtils.addAll(tf_total1.getNumerator(), tf_total1.getDenominator());
      double[] numAndDenOutput2 = ArrayUtils.addAll(tf_total2.getNumerator(), tf_total2.getDenominator());

      // Compare Matlab and Method's outputs.
      assertArrayEquals(numAndDenOutput1, numAndDenOutput2);
   }
}
