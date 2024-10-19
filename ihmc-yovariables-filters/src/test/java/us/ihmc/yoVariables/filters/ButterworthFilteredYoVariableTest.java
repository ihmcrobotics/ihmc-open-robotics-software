package us.ihmc.yoVariables.filters;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.yoVariables.filters.ButterworthFilteredYoVariable.ButterworthFilterType;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ButterworthFilteredYoVariableTest
{
   @Test
   public void testAlphaCompute()
   {
      Random random = new Random(0734454);
      double epsilon = 1.0e-12;

      for (int i = 0; i < 5000; i++)
      {
         double dt = RandomNumbers.nextDouble(random, 1.0e-4, 1.0e-2);
         double samplingFrequency = 1.0 / dt;

         double breakFrequencyIn, alpha, breakFrequencyOut;

         breakFrequencyIn = 0.0;
         alpha = ButterworthFilteredYoVariable.computeAlphaGivenBreakFrequency(breakFrequencyIn, dt);
         assertEquals(1.0, alpha, epsilon);
         breakFrequencyOut = ButterworthFilteredYoVariable.computeBreakFrequencyGivenAlpha(alpha, dt);
         assertEquals(breakFrequencyIn, breakFrequencyOut, epsilon);

         breakFrequencyIn = RandomNumbers.nextDouble(random, 0.0, 0.25 * samplingFrequency);
         alpha = ButterworthFilteredYoVariable.computeAlphaGivenBreakFrequency(breakFrequencyIn, dt);
         breakFrequencyOut = ButterworthFilteredYoVariable.computeBreakFrequencyGivenAlpha(alpha, dt);
         assertEquals(breakFrequencyIn, breakFrequencyOut, epsilon);

         breakFrequencyIn = RandomNumbers.nextDouble(random, 0.25 * samplingFrequency, 10.0 * samplingFrequency);
         alpha = ButterworthFilteredYoVariable.computeAlphaGivenBreakFrequency(breakFrequencyIn, dt);
         assertEquals(0.0, alpha, epsilon);
         breakFrequencyOut = ButterworthFilteredYoVariable.computeBreakFrequencyGivenAlpha(alpha, dt);
         assertEquals(0.25 * samplingFrequency, breakFrequencyOut, epsilon);
      }
   }

   @Test
   public void testBreakFrequencyLowPassFilter()
   {
      double dt = 0.001;
      double desiredBreakFrequency = 4.0;
      double alpha = ButterworthFilteredYoVariable.computeAlphaGivenBreakFrequency(desiredBreakFrequency, dt);
      ButterworthFilteredYoVariable butterworthFilteredYoVariable = new ButterworthFilteredYoVariable("test", null, alpha, ButterworthFilterType.LOW_PASS);

      double actualBreakFrequency = findBreakFrequency(butterworthFilteredYoVariable, dt, dt);
      double percentError = Math.abs(actualBreakFrequency - desiredBreakFrequency) / desiredBreakFrequency;
      // TODO After a bunch of reading, I'm pretty sure the break frequency is properly calculated.
      // The algorithm for finding the break frequency seems to be doing the right thing.
      // We get a percent error of 147% for a break frequency of 100Hz, so I'm missing something somewhere...
      assertEquals(0.0, percentError, 0.05);

      //      Random random = new Random(45353);
      //      int numberOfCycles = 20;
      //
      //      for (int i = 0; i < 1000; i++)
      //      {
      //         dt = 0.001;
      //         double samplingFrequency = 1.0 / dt;
      //         desiredBreakFrequency = RandomNumbers.nextDouble(random, 0.0, 0.25 * samplingFrequency);
      //         alpha = ButterworthFilteredYoVariable.computeAlphaGivenBreakFrequency(desiredBreakFrequency, dt);
      //
      //         butterworthFilteredYoVariable = new ButterworthFilteredYoVariable("test", null, alpha, ButterworthFilterType.LOW_PASS);
      //
      //
      //         double testBreakFrequency = desiredBreakFrequency;
      //         int length = (int) Math.max(Math.ceil(numberOfCycles / (testBreakFrequency * dt)) + 1, 1000);
      //
      //         TimedData[] inputCurve = generateInputCurve(length, testBreakFrequency, dt);
      //         TimedData[] outputCurve = getFilteredCurve(inputCurve, butterworthFilteredYoVariable);
      //
      //         inputCurve = Arrays.copyOfRange(inputCurve, inputCurve.length / 2, inputCurve.length);
      //         outputCurve = Arrays.copyOfRange(outputCurve, outputCurve.length / 2, outputCurve.length);
      //
      //         double actualAttenuation = getMagnitudeInDecibels(inputCurve, outputCurve);
      //         double expectedAttenuation = -Math.abs(computePredictedAttenuationInDecibels(desiredBreakFrequency, 1, testBreakFrequency));
      //
      //         assertEquals(expectedAttenuation, actualAttenuation, 10.0 * dt, "Iteration: " + i + ", desiredBreakFrequency: " + desiredBreakFrequency);
      //      }
   }

   public static double computePredictedAttenuationInDecibels(double cutOffFrequency, int filterOrder, double queryFrequency)
   {
      return 10.0 * Math.log10(1.0 + Math.pow(queryFrequency / cutOffFrequency, 2.0 * filterOrder));
   }

   @Disabled // Old code
   @Test
   public void testButterWorth()
   {
      YoRegistry registry = new YoRegistry("Test");
      double dt = 0.001;
      double desiredBreakFrequency = 1.0;
      double alpha = ButterworthFilteredYoVariable.computeAlphaGivenBreakFrequency(desiredBreakFrequency, dt);
      double testBreakFrequency = desiredBreakFrequency;
      System.out.println(ButterworthFilteredYoVariable.computeAlphaGivenBreakFrequency(testBreakFrequency, dt));

      ButterworthFilteredYoVariable butterworthFilteredYoVariable = new ButterworthFilteredYoVariable("test", registry, alpha, ButterworthFilterType.LOW_PASS);

      int numberOfCycles = 6;
      double endTime = (numberOfCycles) / testBreakFrequency;

      TimedData[] inputCurve = generateInputCurve(endTime, testBreakFrequency, dt);
      TimedData[] outputCurve = getFilteredCurve(inputCurve, butterworthFilteredYoVariable);

      TimedData[] clippedInputCurve = Arrays.copyOfRange(inputCurve, inputCurve.length / 2, inputCurve.length);
      TimedData[] clippedOutputCurve = Arrays.copyOfRange(outputCurve, outputCurve.length / 2, outputCurve.length);

      double dB = getMagnitudeInDecibels(clippedInputCurve, clippedOutputCurve);
      System.out.println("dB= " + dB);
      System.out.println(computePredictedAttenuationInDecibels(desiredBreakFrequency, 1, testBreakFrequency));
   }

   private static TimedData[] generateInputCurve(double endTime, double frequency, double dt)
   {
      return generateInputCurve((int) Math.ceil(endTime / dt) + 1, frequency, dt);
   }

   private static TimedData[] generateInputCurve(int numberOfElements, double frequency, double dt)
   {
      TimedData[] inputCurve = new TimedData[numberOfElements];

      double time = 0.0;

      for (int i = 0; i < numberOfElements; i++)
      {
         inputCurve[i] = new TimedData(time, Math.sin(2.0 * Math.PI * frequency * time));
         time += dt;
      }

      return inputCurve;
   }

   private static double findBreakFrequency(ButterworthFilteredYoVariable filter, double dt, double tolerance)
   {
      int numberOfCycles = 10;

      double samplingFrequency = 1.0 / dt;
      double upperBreakFrequency = 0.5 * samplingFrequency;
      double lowerBreakFrequency = 0.0;
      double upper_dB = Double.NEGATIVE_INFINITY;
      double lower_dB = 0.0;

      double cutOff_dB = -computePredictedAttenuationInDecibels(samplingFrequency, 1, samplingFrequency);

      while (Math.abs(upper_dB - lower_dB) > tolerance && Math.abs(upperBreakFrequency - lowerBreakFrequency) > 0.01 * dt)
      {
         double testBreakFrequency = 0.5 * (upperBreakFrequency + lowerBreakFrequency);
         double endTime = (numberOfCycles) / testBreakFrequency;
         TimedData[] inputCurve = generateInputCurve(endTime, testBreakFrequency, dt);
         TimedData[] outputCurve = getFilteredCurve(inputCurve, filter);

         TimedData[] clippedInputCurve = Arrays.copyOfRange(inputCurve, inputCurve.length / 2, inputCurve.length);
         TimedData[] clippedOutputCurve = Arrays.copyOfRange(outputCurve, outputCurve.length / 2, outputCurve.length);

         double dB = getMagnitudeInDecibels(clippedInputCurve, clippedOutputCurve);
         System.out.println("Test breakFrequency: " + testBreakFrequency + ", dB: " + dB);

         if (dB < cutOff_dB)
         {
            upperBreakFrequency = testBreakFrequency;
            upper_dB = dB;
         }
         else
         {
            lowerBreakFrequency = testBreakFrequency;
            lower_dB = dB;
         }
      }

      return 0.5 * (upperBreakFrequency + lowerBreakFrequency);
   }

   private static TimedData[] getFilteredCurve(TimedData[] input, ButterworthFilteredYoVariable butterworthFilteredYoVariable)
   {
      TimedData[] filteredCurve = new TimedData[input.length];

      butterworthFilteredYoVariable.reset();

      for (int i = 0; i < input.length; i++)
      {
         butterworthFilteredYoVariable.update(input[i].value);

         filteredCurve[i] = new TimedData(input[i].time, butterworthFilteredYoVariable.getDoubleValue());
      }

      return filteredCurve;
   }

   private static double plotBodeForAlpha(double alpha, double dt)
   {
      YoRegistry registry = new YoRegistry("Test");

      ButterworthFilteredYoVariable butterworthFilteredYoVariable = new ButterworthFilteredYoVariable("test", registry, alpha, ButterworthFilterType.LOW_PASS);

      double startFreq = 1e-2;
      double endFreq = 0.25 * (1.0 / dt);

      int numberOfTestPoints = 100;

      double deltaFreq = (endFreq - startFreq) / (numberOfTestPoints - 1);

      List<Double> testFrequencies = new ArrayList<>();
      List<Double> attenuations = new ArrayList<>();

      for (double freq = startFreq; freq <= endFreq; freq = freq + deltaFreq)
      {
         int numberOfCycles = 3;
         double endTime = (numberOfCycles) / freq;

         TimedData[] inputCurve = generateInputCurve(endTime, freq, dt);
         TimedData[] outputCurve = getFilteredCurve(inputCurve, butterworthFilteredYoVariable);

         double dB = getMagnitudeInDecibels(inputCurve, outputCurve);

         testFrequencies.add(freq);
         attenuations.add(dB);
      }

      @SuppressWarnings("unused")
      List<Double> testFrequenciesLog = convertToLog(testFrequencies);

      double breakFreq = getBreakFreq(testFrequencies, attenuations);

      return breakFreq;
   }

   private static double getBreakFreq(List<Double> freq, List<Double> attenutation_dB)
   {
      for (int i = 0; i < freq.size(); i++)
      {
         double attenuation = attenutation_dB.get(i);

         if (attenuation < -3.0)
            return freq.get(i);
      }

      return Double.POSITIVE_INFINITY;
   }

   private static ArrayList<Double> convertToLog(List<Double> arrayListToConvert)
   {
      ArrayList<Double> ret = new ArrayList<>();

      for (Double value : arrayListToConvert)
      {
         double logValue = Math.log10(value);
         ret.add(logValue);
      }

      return ret;
   }

   private static double getMagnitudeInDecibels(TimedData[] input, TimedData[] output)
   {
      double inputAmp = getMaximumPeakToPeakAmplitude(input);
      double outputAmp = getMaximumPeakToPeakAmplitude(output);
      double attenuation = outputAmp / inputAmp;
      return 10.0 * Math.log10(attenuation);
   }

   private static double getMaximumPeakToPeakAmplitude(TimedData[] dataset)
   {
      double value = dataset[0].value;
      double maximumValue = value;
      double minimumValue = value;

      for (int i = 1; i < dataset.length; i++)
      {
         value = dataset[1].value;

         if (value > maximumValue)
            maximumValue = value;
         else if (value < minimumValue)
            minimumValue = value;
      }

      return maximumValue - minimumValue;
   }

   private static class TimedData
   {
      private double time;
      private double value;

      public TimedData(double time, double value)
      {
         this.time = time;
         this.value = value;
      }
   }
}
