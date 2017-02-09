package us.ihmc.avatar.sensors.microphone;

import us.ihmc.robotics.linearDynamicSystems.BodeUnitsConverter;
import us.ihmc.simulationconstructionset.gui.BodePlotConstructor;

public class DrillDetectionAlgorithmSimple extends DrillDetectionAlgorithm
{
   private static final double decibelsOnThreshold = 80.0; //dB
   private static final double frequencyBandLowerBound = 7000; //Hz
   private static final double frequencyBandUpperBound = 9000; //Hz

   @Override
   public DrillDetectionResult isDrillOn(byte[] audioBytes, int size)
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
         time[i] = (double) i / getSampleRate();
      }

      double[][] fftData = BodePlotConstructor.computeFreqMagPhase(time, input);
      double[] frequency = fftData[0];
      double[] magnitude = BodeUnitsConverter.convertMagnitudeToDecibels(fftData[1]);

      //Peak detection math
      int frequencyBandLowerBoundIndex = 0;
      int frequencyBandUpperBoundIndex = 0;

      for (int index = 0; index < frequency.length; index++)
      {
         if (frequency[index] <= frequencyBandLowerBound)
         {
            frequencyBandLowerBoundIndex = index;
         }

         if (frequency[index] <= frequencyBandUpperBound)
         {
            frequencyBandUpperBoundIndex = index;
         }
      }

      double bandAverageMag = 0;
      int rangeSize = frequencyBandUpperBoundIndex - frequencyBandLowerBoundIndex;

      for (int index = 0; index < rangeSize; index++)
      {
         bandAverageMag += magnitude[frequencyBandLowerBoundIndex + index];
      }
      bandAverageMag /= rangeSize;

      DrillDetectionResult result = new DrillDetectionResult();
      result.isOn = (bandAverageMag >= decibelsOnThreshold);
      result.averageValues = new double[] { bandAverageMag };
      result.bodeData = getBodeData(time, input);

      return result;
   }

   @Override
   public int getNumReturnedBands()
   {
      return 1;
   }
}