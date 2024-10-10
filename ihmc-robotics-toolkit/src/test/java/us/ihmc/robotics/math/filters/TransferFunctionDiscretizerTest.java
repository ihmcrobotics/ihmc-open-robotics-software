package us.ihmc.robotics.math.filters;

import org.apache.commons.lang3.ArrayUtils;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class TransferFunctionDiscretizerTest
{

   private double delta = 0.0000000001;

   /*
    * ----------------------- TransferFunctionDiscretizer Tests -----------------------
    */

   /*
    * Check that we cannot create a non-causal filter (m <= n must be true).
    */
   @Test
   void testCausalityCheck()
   {
      assertThrows(IllegalArgumentException.class, () ->
      {
         // Create noncausal tf and try to build filter.
         ContinuousTransferFunction tf = new ContinuousTransferFunction("noncausal_filter", 1.0, new double[] {1.0, 1.0, 1.0}, new double[] {1.0, 1.0});

         TransferFunctionDiscretizer noncausal = new TransferFunctionDiscretizer(tf, 100.0);
      });
   }

   /*
    *  Check accuracy of input/output coefficients for a Low-Pass Filter compared to Matlab output of the coefficients using Tustin's method.
    */
   @Test
   void testCoefficientAccuracy_Order1LPFilter()
   {
      String name = "First Order Low-Pass Filter";
      double Tau_folp = 2 * Math.PI * 10.0;
      ContinuousTransferFunction tf = new ContinuousTransferFunction(name, 1.0, new double[] {1.0}, new double[] {1 / Tau_folp, 1.0});
      TransferFunctionDiscretizer FOLPFilter = new TransferFunctionDiscretizer(tf, 1000.0);

      // Combine input/output vectors into the same vectors for comparison.
      double[] combinedCoefficientsArray = ArrayUtils.addAll(FOLPFilter.getInputCoefficients().getData(), FOLPFilter.getOutputCoefficients().getData());

      // Expected Input and Output Coefficients are validated using c2d in Matlab with the 'Tustin' option (note numerator = input and output = -1*denominator from Matlab's output).
      double[] inputCoeffs = new double[] {0.030459027951421, 0.030459027951421};
      double[] outputCoeffs = new double[] {0.939081944097158};
      double[] combinedExpectedCoeffsArray = ArrayUtils.addAll(inputCoeffs, outputCoeffs);

      assertArrayEquals(combinedExpectedCoeffsArray, combinedCoefficientsArray, delta, name + "input coefficients not equivalent to expected.");
   }

   /*
    *  Check accuracy of input/output coefficients for a 2nd Order Low-Pass Butterworth Filter compared to Matlab output of the coefficients using Tustin's method.
    */
   @Test
   void testCoefficientAccuracy_Order2LPButterFilter()
   {
      String name = "Second Order Low-Pass Butterworth Filter";
      double wc = 2 * Math.PI * 10.0;
      ContinuousTransferFunction tf = new ContinuousTransferFunction(name, 1.0, new double[] {wc * wc}, new double[] {1.0, Math.sqrt(2) * wc, wc * wc});

      TransferFunctionDiscretizer LP_2nd_Order_Butter_Filter = new TransferFunctionDiscretizer(tf, 1000.0);

      // Combine input/output vectors into the same vectors for comparison.
      double[] combinedCoefficientsArray = ArrayUtils.addAll(LP_2nd_Order_Butter_Filter.getInputCoefficients().getData(),
                                                             LP_2nd_Order_Butter_Filter.getOutputCoefficients().getData());

      // Expected Input and Output Coefficients are validated using c2d in Matlab with the 'Tustin' option (note numerator = input and output = -1*denominator from Matlab's output).
      double[] inputCoeffs = new double[] {9.440841143955488e-04, 0.001888168228791, 9.440841143955488e-04};
      double[] outputCoeffs = new double[] {1.911226230340914, -0.915002566798496};
      double[] combinedExpectedCoeffsArray = ArrayUtils.addAll(inputCoeffs, outputCoeffs);

      assertArrayEquals(combinedExpectedCoeffsArray, combinedCoefficientsArray, delta, name + "input coefficients not equivalent to expected.");
   }

   /*
    *  Check accuracy of input/output coefficients for a Low-Pass Filter compared to Matlab output of the coefficients using Tustin's method.
    */
   @Test
   void testCoefficientAccuracy_Order2NotchFilter()
   {
      String name = "Second Order Notch Filter";
      double wn = 60 * 2 * Math.PI;
      double Q = 5.0;

      ContinuousTransferFunction tf = new ContinuousTransferFunction(name, 1.0, new double[] {1.0, 0.0, wn * wn}, new double[] {1.0, wn / Q, wn * wn});

      TransferFunctionDiscretizer Notch_Filter = new TransferFunctionDiscretizer(tf, 1000.0);

      // Combine input/output vectors into the same vectors for comparison.
      double[] combinedCoefficientsArray = ArrayUtils.addAll(Notch_Filter.getInputCoefficients().getData(), Notch_Filter.getOutputCoefficients().getData());

      // Expected Input and Output Coefficients are validated using c2d in Matlab with the 'Tustin' option (note numerator = input and output = -1*denominator from Matlab's output).
      double[] inputCoeffs = new double[] {0.964873211880370, -1.797321552359740, 0.964873211880370};
      double[] outputCoeffs = new double[] {1.797321552359740, -0.929746423760740};
      double[] combinedExpectedCoeffsArray = ArrayUtils.addAll(inputCoeffs, outputCoeffs);

      assertArrayEquals(combinedExpectedCoeffsArray, combinedCoefficientsArray, delta, name + "input coefficients not equivalent to expected.");
   }

   /*
    *  Check accuracy of input/output coefficients for a Complex Multi-order Filter compared to Matlab output of the coefficients using Tustin's method.
    */
   @Test
   void testCoefficientAccuracy_ComplexMultiorderFilter()
   {
      String name = "Complex Multi-Order Filter";
      ContinuousTransferFunction tf = new ContinuousTransferFunction(name,
                                                                     1.0,
                                                                     new double[] {196.919515374308,
                                                                                   21033.790696845190,
                                                                                   427573.897431703983,
                                                                                   18317222.932339027524},
                                                                     new double[] {1.000000000000, 382.156022138851, 60851.343857079330, 3875784.585037478711});
      TransferFunctionDiscretizer InvPlant_N_to_mA_Filter = new TransferFunctionDiscretizer(tf, 1000.0);

      // Combine input/output vectors into the same vectors for comparison.
      double[] combinedCoefficientsArray = ArrayUtils.addAll(InvPlant_N_to_mA_Filter.getInputCoefficients().getData(),
                                                             InvPlant_N_to_mA_Filter.getOutputCoefficients().getData());

      // Expected Input and Output Coefficients are validated using c2d in Matlab with the 'Tustin' option (note numerator = input and output = -1*denominator from Matlab's output).
      double[] inputCoeffs = new double[] {1.719836247824349e+02, -4.981554304451415e+02, 4.807370656265892e+02, -1.545500813116288e+02};
      double[] outputCoeffs = new double[] {2.630491113793050, -2.316224508086777, 0.682521707312347};
      double[] combinedExpectedCoeffsArray = ArrayUtils.addAll(inputCoeffs, outputCoeffs);

      assertArrayEquals(combinedExpectedCoeffsArray, combinedCoefficientsArray, delta, name + "input coefficients not equivalent to expected.");
   }

   /*
    *  Check accuracy of input/output coefficients for a PID Controller compared to Matlab output of the coefficients using Tustin's method.
    */
   @Test
   void testCoefficientAccuracy_PIDController()
   {
      String name = "PID Controller";
      double Kp = 15.0;
      double Ki = 2.0;
      double Kd = 0.25;
      double Tau = 0.0035;

      ContinuousTransferFunction tf = new ContinuousTransferFunction(name,
                                                                     1.0,
                                                                     new double[] {(Kp + Tau * Kd), (Tau * Kp + Ki), Ki * Tau},
                                                                     new double[] {1.0, Tau, 0.0});

      TransferFunctionDiscretizer PID_Filter = new TransferFunctionDiscretizer(tf, 1000.0);

      // Combine input/output vectors into the same vectors for comparison.
      double[] combinedCoefficientsArray = ArrayUtils.addAll(PID_Filter.getInputCoefficients().getData(), PID_Filter.getOutputCoefficients().getData());

      // Expected Input and Output Coefficients are validated using c2d in Matlab with the 'Tustin' option (note numerator = input and output = -1*denominator from Matlab's output).
      double[] inputCoeffs = new double[] {15.001874998468752, -30.001697493529385, 14.999822502060621};
      double[] outputCoeffs = new double[] {1.999996500006125, -0.999996500006125};
      double[] combinedExpectedCoeffsArray = ArrayUtils.addAll(inputCoeffs, outputCoeffs);

      assertArrayEquals(combinedExpectedCoeffsArray, combinedCoefficientsArray, delta, name + "input coefficients not equivalent to expected.");
   }

   /*
    *  Check accuracy of input/output coefficients for a PID Controller compared to Matlab output of the coefficients using Tustin's method.
    */
   @Test
   void testCoefficientAccuracy_LeadLagController()
   {
      String name = "Lead-Lag Controller";
      double k = 10;
      double z = 2 * Math.PI * 1;
      double p = 2 * Math.PI * 10;

      ContinuousTransferFunction tf = new ContinuousTransferFunction(name, k, new double[] {1.0, z}, new double[] {1.0, p});

      TransferFunctionDiscretizer Lead_Lag_Compensator_Filter = new TransferFunctionDiscretizer(tf, 1000.0);

      // Combine input/output vectors into the same vectors for comparison.
      double[] combinedCoefficientsArray = ArrayUtils.addAll(Lead_Lag_Compensator_Filter.getInputCoefficients().getData(),
                                                             Lead_Lag_Compensator_Filter.getOutputCoefficients().getData());

      // Expected Input and Output Coefficients are validated using c2d in Matlab with the 'Tustin' option (note numerator = input and output = -1*denominator from Matlab's output).
      double[] inputCoeffs = new double[] {9.725868748437209, -9.664950692534367};
      double[] outputCoeffs = new double[] {0.939081944097158};
      double[] combinedExpectedCoeffsArray = ArrayUtils.addAll(inputCoeffs, outputCoeffs);

      assertArrayEquals(combinedExpectedCoeffsArray, combinedCoefficientsArray, delta, name + "input coefficients not equivalent to expected.");
   }
}

