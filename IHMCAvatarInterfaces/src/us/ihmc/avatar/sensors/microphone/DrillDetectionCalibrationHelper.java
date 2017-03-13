package us.ihmc.avatar.sensors.microphone;

import java.util.ArrayList;

import us.ihmc.commons.Conversions;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.simulationconstructionset.gui.BodePlotConstructor;

/**
 * <p>Description: Detects a distinct sound by searching for a characteristic peak in FFT magnitude data of sound data
 * from the Atlas Chest Webcam microphone around a given frequency</p>
 */
public class DrillDetectionCalibrationHelper extends DrillDetectionAlgorithm
{

   private static final double decibelsDeltaToTripDetection = -5.8; //dB
   private static final double lowerFrequencyBound = 1000;
   private static final double upperFrequencyBound = 8000;
   private static final double frequencyBandRange = 1000;

   private YoVariableRegistry registry = new YoVariableRegistry("DrillRegistry");
   private ArrayList<DoubleYoVariable> rawBandMagnitudes = new ArrayList<>();
   private ArrayList<AlphaFilteredYoVariable> filteredBandMagnitudes = new ArrayList<>();

   public DrillDetectionCalibrationHelper(){
      int numberOfBands = (int)  ((upperFrequencyBound - lowerFrequencyBound) / frequencyBandRange);

      double upperFrequency;
      double lowerFrequency;
      for (int i = 0; i < numberOfBands; i++){
         upperFrequency = upperFrequencyBound - i*frequencyBandRange;
         lowerFrequency = upperFrequency - frequencyBandRange;
         rawBandMagnitudes.add(new DoubleYoVariable("raw"+(int)lowerFrequency+""+(int)upperFrequency+"BandMagnitude", registry));
         filteredBandMagnitudes.add(new AlphaFilteredYoVariable("filtered"+(int)lowerFrequency+""+(int)upperFrequency+"BandMagnitude", registry, 0.9));
      }
   }

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
      double[] magnitude = Conversions.convertMagnitudeToDecibels(fftData[1]);


      double lowerFrequency;
      double upperFrequency;
      int lowerIndex = 0;
      int upperIndex = 0;

      int numberOfBands = rawBandMagnitudes.size();
      for (int i = 0; i < numberOfBands; i++){
         lowerFrequency = upperFrequencyBound - (numberOfBands - i) * frequencyBandRange;
         upperFrequency = lowerFrequency + frequencyBandRange;
         for (int index = 0; index < frequency.length; index++){
            if (frequency[index] <= lowerFrequency){
               lowerIndex = index;
            }
            if (frequency[index] <= upperFrequency){
               upperIndex = index;
            }
         }
         double bandMagnitude = 0;
         for (int index = lowerIndex; index < upperIndex; index++){
            bandMagnitude += magnitude[index];
         }
         if (upperIndex - lowerIndex > 0){
            bandMagnitude /= (double)(upperIndex - lowerIndex);
         }
         rawBandMagnitudes.get(i).set(bandMagnitude);
         filteredBandMagnitudes.get(i).update(bandMagnitude);
      }

      DrillDetectionResult result = new DrillDetectionResult();
      result.isOn = false;
      result.averageValues = new double[numberOfBands];
      for (int i = 0; i < numberOfBands; i++){
//         result.averageValues[i] = rawBandMagnitudes.get(i).getDoubleValue();
         result.averageValues[i] = filteredBandMagnitudes.get(i).getDoubleValue();
      }
      result.bodeData = getBodeData(time, input);

      return result;
   }

   @Override
   public int getNumReturnedBands()
   {
      return rawBandMagnitudes.size();
   }
}