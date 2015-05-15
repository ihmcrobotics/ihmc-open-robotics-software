package us.ihmc.darpaRoboticsChallenge.sensors.microphone;

import java.io.InputStream;

import javax.sound.sampled.AudioFormat;

import us.ihmc.simulationconstructionset.gui.BodePlotConstructor;
import us.ihmc.utilities.linearDynamicSystems.BodeUnitsConverter;

/**
 * <p>Description: Detects a distinct sound by searching for a characteristic peak in FFT magnitude data of sound data
 * from the Atlas Chest Webcam microphone around a given frequency</p>
 */
public class WorkingDrillDetector
{

   /**
    * The relevant frequency band is the range of frequencies that are used for comparison to distinguish a peak.
    * The magnitude data is averaged across this range with the exception of the dominant frequency band magnitudes.
    * <p/>
    * The dominant frequency band is the range of frequencies were the peak in magnitude could show up in. This band must be within the relevant band.
    * <p/>
    * The decibelsDeltaToTripDetection is the minimum difference in magnitude the peak must have over the average magnitude to be considered a significant
    * enough peak to trip detection.
    */

   private static final double decibelsDeltaToTripDetection = -5.5; //dB
   private static final double dominantFrequencyBandLowerBound = 6900; //Hz
   private static final double dominantFrequencyBandUpperBound = 7900; //Hz
   private static final double relevantFrequencyBandLowerBound = 1000; //Hz
   private static final double relevantFrequencyBandUpperBound = 4900; //Hz

   private static final float sampleRate = 16000; //Hz
   private static final int sampleSizeInBits = 16;
   private static final int channels = 1;
   private static final boolean signed = true;
   private static final boolean bigEndian = false;
   private static final AudioFormat format = new AudioFormat(sampleRate, sampleSizeInBits, channels, signed, bigEndian);
   private static final int frameSizeInBytes = format.getFrameSize();
   private static final int bufferLengthInFrames = 16384 / 8;
   private static final int bufferLengthInBytes = bufferLengthInFrames * frameSizeInBytes;

   public DrillDetectionResult isDrillOn(InputStream inputStream)
   {
      if (format.getSampleSizeInBits() != 16)
      {
         System.out.println("Can't detect: bad sample size");
         return null;
      }

      try
      {
         byte[] data = new byte[bufferLengthInBytes];
         int numBytesRead = inputStream.read(data, 0, bufferLengthInBytes);
         return detectDrill(data, numBytesRead);
      }
      catch (Exception ignored)
      {
         System.out.println("Failed to read from stream...");
         return null;
      }
   }

   private DrillDetectionResult detectDrill(byte[] audioBytes, int size)
   {
      int nlengthInSamples = size / 2;
      int[] audioData = new int[nlengthInSamples];

      //Little-Endian code
      for (int i = 0; i < nlengthInSamples; i++)
      {
         int LSB = (int) audioBytes[2 * i];     /* First byte is LSB (low order) */
         int MSB = (int) audioBytes[2 * i + 1]; /* Second byte is MSB (high order) */
         audioData[i] = (MSB << 8) | (LSB & 0xFF);
      }

      double[] input = new double[audioData.length];
      double[] time = new double[audioData.length];
      for (int i = 0; i < audioData.length; i++)
      {
         input[i] = (double) audioData[i];
         time[i] = (double) i / format.getSampleRate();
      }

      double[][] fftData = BodePlotConstructor.computeFreqMagPhase(time, input);
      double[] frequency = fftData[0];
      double[] magnitude = BodeUnitsConverter.convertMagnitudeToDecibels(fftData[1]);

      //Peak detection math
      int dominantFrequencyBandLowerBoundIndex = 0;
      int dominantFrequencyBandUpperBoundIndex = 0;
      int relevantFrequencyBandLowerBoundIndex = 0;
      int relevantFrequencyBandUpperBoundIndex = 0;

      for (int index = 0; index < frequency.length; index++)
      {
         if (frequency[index] <= dominantFrequencyBandLowerBound)
         {
            dominantFrequencyBandLowerBoundIndex = index;
         }

         if (frequency[index] <= dominantFrequencyBandUpperBound)
         {
            dominantFrequencyBandUpperBoundIndex = index;
         }

         if (frequency[index] <= relevantFrequencyBandLowerBound)
         {
            relevantFrequencyBandLowerBoundIndex = index;
         }

         if (frequency[index] <= relevantFrequencyBandUpperBound)
         {
            relevantFrequencyBandUpperBoundIndex = index;
         }
      }

      double dominantBandAverageMag = 0;
      double relevantBandAverageMag = 0;

      int dominantRangeSize = dominantFrequencyBandUpperBoundIndex - dominantFrequencyBandLowerBoundIndex;
      int relevantRangeSize = relevantFrequencyBandUpperBoundIndex - relevantFrequencyBandLowerBoundIndex;

      for (int index = 0; index < relevantRangeSize; index++)
      {
         relevantBandAverageMag += magnitude[relevantFrequencyBandLowerBoundIndex + index];
      }
      relevantBandAverageMag /= relevantRangeSize;

      for (int index = 0; index < dominantRangeSize; index++)
      {
         dominantBandAverageMag += magnitude[dominantFrequencyBandLowerBoundIndex + index];
      }
      dominantBandAverageMag /= dominantRangeSize;

      DrillDetectionResult result = new DrillDetectionResult();
      result.isOn = ((dominantBandAverageMag - relevantBandAverageMag) > decibelsDeltaToTripDetection);
      result.bodeData = getBodeData(time, input);

      return result;
   }

   private double[][] getBodeData(double[] time, double[] data)
   {
      double[][] freqMagPhase = BodePlotConstructor.computeFreqMagPhase(time, data);

      double[] frequency = freqMagPhase[0];
      double[] magnitude = BodeUnitsConverter.convertMagnitudeToDecibels(freqMagPhase[1]);
      double[] phase = BodeUnitsConverter.convertRadianToDegrees(freqMagPhase[2]);

      double[][] bodeData = new double[][] { frequency, magnitude, phase };
      return bodeData;
   }
}