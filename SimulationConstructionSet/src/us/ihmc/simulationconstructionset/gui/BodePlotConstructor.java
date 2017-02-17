package us.ihmc.simulationconstructionset.gui;

import org.jtransforms.fft.DoubleFFT_1D;

import us.ihmc.robotics.linearDynamicSystems.BodeUnitsConverter;
import us.ihmc.robotics.linearDynamicSystems.TransferFunction;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2009</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class BodePlotConstructor
{
   private BodePlotConstructor()
   {
   }

   public static void plotFFT(String variableName, double[] time, double[] data)
   {
      double[][] freqMagPhase = computeFreqMagPhase(time, data);

      double[] frequency = freqMagPhase[0];
      double[] magnitude = BodeUnitsConverter.convertMagnitudeToDecibels(freqMagPhase[1]);
      double[] phase = BodeUnitsConverter.convertRadianToDegrees(freqMagPhase[2]);

      double[][] bodeData = new double[][]
      {
         frequency, magnitude, phase
      };

      FFTPlotter plot = new FFTPlotter(bodeData, variableName + " FFT Plot", "(Hz)", "(dB)", "(deg)");
      plot.packAndDisplayFrame(0, 0);
   }


   public static void plotBodeFromInputToOutput(String inputName, String outputName, double[] time, double[] input, double[] output)
   {
      double[][] bodeFreqMagPhase = computeBodeFreqMagPhaseFromInputToOutput(time, input, output);

      double[] bodeFrequency = bodeFreqMagPhase[0];
      double[] bodeMagnitude = bodeFreqMagPhase[1];
      double[] bodePhase = bodeFreqMagPhase[2];

      double[][] bodeData = new double[][]
      {
         bodeFrequency, bodeMagnitude, bodePhase
      };

      FFTPlotter plot = new FFTPlotter(bodeData, outputName + "/" + inputName + " Bode Plot", "(Hz)", "(dB)", "(deg)");
      plot.packAndDisplayFrame(0, 0);
   }

   public static void plotBodeForTransferFunction(String name, TransferFunction transferFunction, double[] omega)
   {
      double[] bodeMagnitude = transferFunction.getMagnitude(omega);
      bodeMagnitude = BodeUnitsConverter.convertMagnitudeToDecibels(bodeMagnitude);

      double[] bodePhase = transferFunction.getPhase(omega);
      bodePhase = BodeUnitsConverter.convertRadianToDegrees(bodePhase);

      double[] bodeFrequency = BodeUnitsConverter.convertRadPerSecondToHz(omega);

      double[][] bodeData = new double[][]
      {
         bodeFrequency, bodeMagnitude, bodePhase
      };

      FFTPlotter plot = new FFTPlotter(bodeData, name + " Bode Plot", "(Hz)", "(dB)", "(deg)");
      plot.packAndDisplayFrame(0, 0);
   }

   public static double[][] computeBodeFreqMagPhaseFromInputToOutput(double[] time, double[] input, double[] output)
   {
      int n = time.length;

      if (input.length != n)
         throw new RuntimeException("input.length != n");
      if (output.length != n)
         throw new RuntimeException("output.length != n");

      double[][] inputFreqMagPhase = computeFreqMagPhase(time, input);
      double[][] outputFreqMagPhase = computeFreqMagPhase(time, output);

      double[] frequency = inputFreqMagPhase[0];
      double[] inputMagnitude = inputFreqMagPhase[1];
      double[] inputPhase = inputFreqMagPhase[2];
      double[] outputMagnitude = outputFreqMagPhase[1];
      double[] outputPhase = outputFreqMagPhase[2];

      double[] bodeMagnitude = divide(outputMagnitude, inputMagnitude);
      bodeMagnitude = BodeUnitsConverter.convertMagnitudeToDecibels(bodeMagnitude);

      double[] bodePhase = subtractMod2PI(outputPhase, inputPhase);
      bodePhase = BodeUnitsConverter.convertRadianToDegrees(bodePhase);

      return new double[][]
      {
         frequency, bodeMagnitude, bodePhase
      };
   }

   public static double[][] computeFreqMagPhase(double[][] timeAndData)
   {
      return computeFreqMagPhase(timeAndData[0], timeAndData[1]);
   }

   public static double[][] computeFreqMagPhase(double[] time, double[] data)
   {
      int n = time.length;
      if (data.length != n)
         throw new RuntimeException("input.length != n");

      DoubleFFT_1D fft = new DoubleFFT_1D(n);

      double T = time[n - 1] - time[0];

      double fftInput[] = copyDoubleArray(data);

      fft.realForward(fftInput);

      double[] magnitude = extractMagnitude(fftInput);
      double[] phase = extractPhaseRadians(fftInput);
      double[] frequency = new double[n / 2];
      for (int i = 0; i < n / 2; i++)
      {
         frequency[i] = (i) / T;
      }

      return new double[][]
      {
         frequency, magnitude, phase
      };
   }

   @SuppressWarnings("unused")
   private static void print(double[] a)
   {
      int n = a.length;

      for (int i = 0; i < n; i++)
      {
         System.out.println(a[i]);
      }
   }

   private static double[] divide(double[] numerator, double[] denominator)
   {
      int n = numerator.length;
      if (denominator.length != n)
         throw new RuntimeException("a and b must be same length!");

      double[] ret = new double[n];

      for (int i = 0; i < n; i++)
      {
         ret[i] = numerator[i] / denominator[i];
      }

      return ret;
   }

   private static double[] subtractMod2PI(double[] a, double[] b)
   {
      int n = a.length;
      if (b.length != n)
         throw new RuntimeException("a and b must be same length!");

      double[] ret = new double[n];

      for (int i = 0; i < n; i++)
      {
         ret[i] = computeAngleDifferenceMinusTwoPiToZero(a[i], b[i]);

//       ret[i] = MathTools.computeAngleDifferenceMinusPiToPi(a[i], b[i]);
         // ret[i] = a[i] - b[i];
      }

      return ret;
   }

   private static double computeAngleDifferenceMinusTwoPiToZero(double angleA, double angleB)
   {
      double difference = angleA - angleB;
      difference = difference % (Math.PI * 2.0);
      difference = shiftAngleToStartOfRange(difference, -1.0 * Math.PI);

      return difference;
   }

   private static double shiftAngleToStartOfRange(double angleToShift, double startOfAngleRange)
   {
      double ret = angleToShift;
      startOfAngleRange = startOfAngleRange - 1e-7;

      if (angleToShift < startOfAngleRange)
      {
         ret = angleToShift + Math.ceil((startOfAngleRange - angleToShift) / (2.0 * Math.PI)) * Math.PI * 2.0;
      }

      if (angleToShift >= (startOfAngleRange + Math.PI * 2.0))
      {
         ret = angleToShift - Math.floor((angleToShift - startOfAngleRange) / (2.0 * Math.PI)) * Math.PI * 2.0;
      }

      return ret;
   }


   private static double[] extractMagnitude(double[] fftInput)
   {
      int n = fftInput.length;

//    if (n % 2 != 0)
//        throw new RuntimeException("Must be even length!");

      double[] magnitude = new double[n / 2];

      for (int k = 0; k < n / 2; k++)
      {
         double real = fftInput[2 * k];
         double imag = fftInput[2 * k + 1];

         if (k == 0)
            imag = 0;    // This is due to storing a[1] = Re[n/2] when n is even and = Im[n/2-1] when n is odd, as per the fft specs. Just ignore that term for now...

         double mag = Math.sqrt(real * real + imag * imag);
         magnitude[k] = mag;

//       magnitude[k] = 20.0 * Math.log10(mag);
      }

      return magnitude;
   }

   private static double[] extractPhaseRadians(double[] fftInput)
   {
      int n = fftInput.length;

//    if (n % 2 != 0)
//        throw new RuntimeException("Must be even length!");

      double[] phase = new double[n / 2];

      for (int k = 0; k < n / 2; k++)
      {
         double real = fftInput[2 * k];
         double imag = fftInput[2 * k + 1];

         if (k == 0)
            imag = 0;    // This is due to storing a[1] = Re[n/2] when n is even and = Im[n/2-1] when n is odd, as per the fft specs. Just ignore that term for now...

         phase[k] = Math.atan2(imag, real);
      }

      return phase;
   }

   private static double[] copyDoubleArray(double[] in)
   {
      double[] ret = new double[in.length];

      for (int i = 0; i < in.length; i++)
      {
         ret[i] = in[i];
      }

      return ret;
   }



}
