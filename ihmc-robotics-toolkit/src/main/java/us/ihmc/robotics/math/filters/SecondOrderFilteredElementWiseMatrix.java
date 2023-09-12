package us.ihmc.robotics.math.filters;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SecondOrderFilteredElementWiseMatrix
{
   private static int FILTER_ORDER = 2;

   private final DMatrixRMaj[] outputs = new DMatrixRMaj[FILTER_ORDER + 1]; // magic number of 3 because it's a second-order filter

   private final DMatrixRMaj[] inputs = new DMatrixRMaj[FILTER_ORDER + 1];

   private final DMatrixRMaj filtered;

   private final double samplingFrequency;

   private final double[] aCoeffs = new double[3];  // magic number of 3 because it's a second-order filter
   private final double[] bCoeffs = new double[3];

   public SecondOrderFilteredElementWiseMatrix(String name, int numberOfRows, int numberOfColumns, double samplingFrequency, double cutoffFrequency, YoRegistry registry)
   {
      this.samplingFrequency = samplingFrequency;

      filtered = new DMatrixRMaj(numberOfRows, numberOfColumns);

      for (int i = 0; i < FILTER_ORDER + 1; ++i)  // magic number of 3 because it's a second-order filter
      {
         outputs[i] = new DMatrixRMaj(numberOfRows, numberOfColumns);
         inputs[i] = new DMatrixRMaj(numberOfRows, numberOfColumns);
      }

      SecondOrderLowPassDigitalFilterCalculator.calculatefilterCoefficients(samplingFrequency, cutoffFrequency, bCoeffs, aCoeffs);
   }

   public void setCutoffFrequency(double cutoffFrequency)
   {
      SecondOrderLowPassDigitalFilterCalculator.calculatefilterCoefficients(samplingFrequency, cutoffFrequency, bCoeffs, aCoeffs);
   }

   public void setAndSolve(DMatrixRMaj current)
   {
      for (int i = FILTER_ORDER; i > 0; --i)
      {
         inputs[i].set(inputs[i - 1]);
         outputs[i].set(outputs[i - 1]);
      }
      inputs[0].set(current);

      for (int j = 0; j < current.getNumElements(); ++j)
      {
         double value = 0.0;
         // The most recent inputs/outputs are at the top, that's also how the filter coefficients are ordered
         for (int coeffIndex = 0; coeffIndex < FILTER_ORDER + 1; ++coeffIndex)
         {
            // Because it's just a linear sum, there's nothing stopping us adding in the a and b filter contributions in an unordered manner
            value += bCoeffs[coeffIndex] * inputs[coeffIndex].get(j);
            if (coeffIndex != 0)
               value -= aCoeffs[coeffIndex] * outputs[coeffIndex].get(j);
         }
         outputs[0].set(j, value);
      }
      filtered.set(outputs[0]);
   }

   public DMatrixRMaj getFilteredMatrix()
   {
      return filtered;
   }

   // TODO(jfoster): Could probably get filtered out to something like FilterTools
   public static class SecondOrderLowPassDigitalFilterCalculator
   {
      /**
       * Design a second order low pass filter given a sampling frequency and desired cutoff frequency
       */
      // TODO(jfoster): Assuming a butterworth filter only at the moment
      public static void calculatefilterCoefficients(double samplingFrequencyInHz, double cutoffFrequencyInHz, double[] bCoefficientsToPack, double[] aCoefficientsToPack)
      {
         if (cutoffFrequencyInHz > 0.5 * samplingFrequencyInHz)
         {
            LogTools.warn("Cutoff frequency of filter (" + cutoffFrequencyInHz + " Hz) is larger than nyquist frequency (" + samplingFrequencyInHz / 2 + " Hz), beware of aliasing");
         }

         // Introduce mathematial notation to make things more compact
         double samplingTime = 1.0 / samplingFrequencyInHz;
         double cutoffFrequencyInRads = 2 * Math.PI * cutoffFrequencyInHz;
         double W = Math.tan(cutoffFrequencyInRads * samplingTime / 2.0);
         double dampingRatio = Math.sqrt(2);  // Damping ratio of a butterworth filter

         // See https://thewolfsound.com/bilinear-transform/ for calculation
         bCoefficientsToPack[0] = W * W;
         bCoefficientsToPack[1] = 2.0 * W * W;
         bCoefficientsToPack[2] = W * W;

         aCoefficientsToPack[0] = 1.0 + W * dampingRatio + W * W;
         aCoefficientsToPack[1] = 2 * (W * W - 1);
         aCoefficientsToPack[2] = W * W - W * dampingRatio + 1;

         // Normalize results by first entry of aCoeffs (this is the value that multiplies the current input of the filter, best set to 1)
         double normalizeValue = aCoefficientsToPack[0];
         for (int i = 0; i < FILTER_ORDER + 1; ++i)
         {
            bCoefficientsToPack[i] /= normalizeValue;
            aCoefficientsToPack[i] /= normalizeValue;
         }

      }
   }
}
