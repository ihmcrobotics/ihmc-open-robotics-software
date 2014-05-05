package us.ihmc.darpaRoboticsChallenge.sensorProcessing;

import boofcv.alg.transform.fft.GeneralPurposeFFT_F64_1D;

/**
 * @author Peter Abeles
 */
public class AtlasWirstBasedDrillDetector
{
   int windowSize;
   int numWindows;
   double threshold;

   int count = 0;
   int countWindow = 0;
   double data[];
   double magnitude[];
   
   boolean detected = false;

   GeneralPurposeFFT_F64_1D fft;

   public AtlasWirstBasedDrillDetector(int windowSize, int numWindows, double threshold)
   {
      this.windowSize = windowSize;
      this.numWindows = numWindows;
      this.threshold = threshold;

      fft = new GeneralPurposeFFT_F64_1D(windowSize);

      data = new double[windowSize * 2];
      magnitude = new double[windowSize];
   }

   public double computeStdev(double data[], int begin, int end)
   {
      double mean = 0;
      for (int i = begin; i < end; i++)
      {
         mean += data[i];
      }
      mean /= 50;
      double stdev = 0;
      for (int i = begin; i < end; i++)
      {
         double d = data[i] - mean;
         stdev += d * d;
      }
      return Math.sqrt(stdev / (end - begin));
   }

   public boolean addMeasurement(double measurement)
   {

      data[count++] = measurement;
      if (count >= windowSize)
      {
         count = 0;
         countWindow++;

         fft.realForwardFull(data);

         for (int j = 0; j < windowSize; j++)
         {
            double real = data[j * 2];
            double img = data[j * 2 + 1];

            magnitude[j] += Math.sqrt(real * real + img * img);
         }

         if (countWindow >= numWindows)
         {
            countWindow = 0;
            detected = false;
            for (int j = 0; j < windowSize; j++)
            {
               magnitude[j] /= numWindows;
            }

            double stdev = computeStdev(magnitude, windowSize * 3 / 8, windowSize * 5 / 8);

            if (stdev > threshold) {
               detected = true;
            }

            for (int j = 0; j < windowSize; j++)
            {
               magnitude[j] = 0;
            }
         }
      }

      return detected;
   }
}
