package us.ihmc.darpaRoboticsChallenge.sensors.microphone;

import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;

import javax.sound.sampled.AudioFormat;

import us.ihmc.simulationconstructionset.gui.BodePlotConstructor;
import us.ihmc.utilities.linearDynamicSystems.BodeUnitsConverter;

/**
 * <p>Title: DrillDetector</p>
 * <p>Description: Detects a distinct sound by searching for a characteristic peak in FFT magnitude data of sound data
 * from the Atlas Chest Webcam microphone around a given frequency</p>
 * 
 * @author Will
 * @author Igor
 */
public class DrillDetector
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

   private static final double decibelsDeltaToTripDetection = 10; //dB
   private static final double dominantFrequencyBandLowerBound = 5700; //Hz
   private static final double dominantFrequencyBandUpperBound = 6000; //Hz
   private static final double relevantFrequencyBandLowerBound = 5000; //Hz
   private static final double relevantFrequencyBandUpperBound = 7000; //Hz

   private static final float sampleRate = 16000; //Hz
   private static final int sampleSizeInBits = 16;
   private static final int channels = 1;
   private static final boolean signed = true;
   private static final boolean bigEndian = false;
   private static final AudioFormat format = new AudioFormat(sampleRate, sampleSizeInBits, channels, signed, bigEndian);
   private static final int frameSizeInBytes = format.getFrameSize();
   private static final int bufferLengthInFrames = 16384 / 8;
   private static final int bufferLengthInBytes = bufferLengthInFrames * frameSizeInBytes;

   private static final String COMMA_DELIMITER = ",";
   private static final String NEW_LINE_SEPARATOR = "\n";

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

      //Write buffer data to a CSV file
//      writeAudioDataToLog(audioData);

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

      //Write frequency and magnitude data to a CSV file
//      writeProcessedDataToLog(frequency, magnitude);

      //Peak detection math
      int dominantFrequencyBandLowerBoundIndex = 0;
      int dominantFrequencyBandUpperBoundIndex = 0;
      int relevantFrequencyBandLowerBoundIndex = 0;
      int relevantFrequencyBandUpperBoundIndex = 0;

      for (int index = 0; frequency[index] < dominantFrequencyBandLowerBound; index++)
      {
         dominantFrequencyBandLowerBoundIndex = index;
      }

      for (int index = 0; frequency[index] < dominantFrequencyBandUpperBound; index++)
      {
         dominantFrequencyBandUpperBoundIndex = index;
      }

      for (int index = 0; frequency[index] < relevantFrequencyBandLowerBound; index++)
      {
         relevantFrequencyBandLowerBoundIndex = index;
      }

      for (int index = 0; frequency[index] < relevantFrequencyBandUpperBound; index++)
      {
         relevantFrequencyBandUpperBoundIndex = index;
      }

      double dominantBandPeakMag = 0;
      double relevantBandAverageMag = 0;

      for (int index = relevantFrequencyBandLowerBoundIndex; index < dominantFrequencyBandLowerBoundIndex; index++)
      {
         relevantBandAverageMag += magnitude[index];
      }

      for (int index = dominantFrequencyBandUpperBoundIndex; index < relevantFrequencyBandUpperBoundIndex; index++)
      {
         relevantBandAverageMag += magnitude[index];
      }

      int temp1 = (relevantFrequencyBandUpperBoundIndex - dominantFrequencyBandUpperBoundIndex);
      int temp2 = (dominantFrequencyBandLowerBoundIndex - relevantFrequencyBandLowerBoundIndex);
      relevantBandAverageMag = relevantBandAverageMag / (temp1 + temp2);

      for (int index = dominantFrequencyBandLowerBoundIndex; index < dominantFrequencyBandUpperBoundIndex; index++)
      {
         if (magnitude[index] > dominantBandPeakMag)
         {
            dominantBandPeakMag = magnitude[index];
         }
      }

      DrillDetectionResult result = new DrillDetectionResult();
      result.isOn = ((dominantBandPeakMag - relevantBandAverageMag) > decibelsDeltaToTripDetection);
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

   private void writeAudioDataToLog(int[] audioData)
   {
      try
      {
         FileWriter fileWriter = new FileWriter("buffercsv.csv");

         for (int i = 0; i < audioData.length; i++)
         {
            String line = i + COMMA_DELIMITER + audioData[i];
            fileWriter.append(line);
            fileWriter.append(NEW_LINE_SEPARATOR);
         }

         fileWriter.flush();
         fileWriter.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void writeProcessedDataToLog(double[] frequency, double[] magnitude)
   {
      try
      {
         FileWriter fileWriterBode = new FileWriter("freq_mag.csv");

         for (int i = 0; i < frequency.length; i++)
         {
            String line = frequency[i] + COMMA_DELIMITER + magnitude[i];
            fileWriterBode.append(line);
            fileWriterBode.append(NEW_LINE_SEPARATOR);
         }

         fileWriterBode.flush();
         fileWriterBode.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}