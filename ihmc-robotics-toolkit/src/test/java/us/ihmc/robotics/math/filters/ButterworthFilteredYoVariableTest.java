package us.ihmc.robotics.math.filters;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.robotics.math.filters.ButterworthFilteredYoVariable.ButterworthFilterType;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ButterworthFilteredYoVariableTest
{

   @Test
   public void testButterWorthFilter()
   {
      boolean visualize = false;

      YoRegistry registry = new YoRegistry("Test");
      double deltaT = 0.002;

      double breakFrequencyInHertz = 10.0;
      double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequencyInHertz, deltaT);

      ButterworthFilteredYoVariable lowPassYoVariable = new ButterworthFilteredYoVariable("lowPass", registry, alpha, ButterworthFilterType.LOW_PASS);
      ButterworthFilteredYoVariable highPassYoVariable = new ButterworthFilteredYoVariable("highPass", registry, alpha, ButterworthFilterType.HIGH_PASS);

      double freq = 10.0;
      int numberOfCycles = 3;
      double endTime = (numberOfCycles) / freq;

      int numberOfElements = (int) Math.ceil(endTime / deltaT) + 1;

      double[][] inputCurve = generateInputCurve(endTime, freq, deltaT);
      double[][] outputCurveLowPass = getFilteredCurve(inputCurve, lowPassYoVariable);
      double[][] outputCurveHighPass = getFilteredCurve(inputCurve, highPassYoVariable);

      double dB = getMagnitudeInDecibels(inputCurve, outputCurveLowPass);
      System.out.println("dB= " + dB);

      if (visualize)
      {
         SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("TestRobot"));
         scs.addYoRegistry(registry);
         
         YoDouble input = new YoDouble("input", registry);
         YoDouble outputLowPass = new YoDouble("outputLowPass", registry);
         YoDouble outputHighPass = new YoDouble("outputHighPass", registry);
         YoDouble outputSum = new YoDouble("outputSum", registry);
         
         scs.startOnAThread();

         
         for (int i=0; i<inputCurve[0].length; i++)
         {
            input.set(inputCurve[1][i]);
            outputHighPass.set(outputCurveHighPass[1][i]);
            outputLowPass.set(outputCurveLowPass[1][i]);
            
            outputSum.set(outputHighPass.getValue() + outputLowPass.getValue());
            
            scs.setTime(inputCurve[0][i]);
            
            scs.tickAndUpdate();
         }

         ThreadTools.sleepForever();
      }
   }

   private static double[][] generateInputCurve(double endTime, double frequency, double deltaT)
   {
      int numberOfElements = (int) Math.ceil(endTime / deltaT) + 1;

      double[][] inputCurve;

      inputCurve = new double[2][numberOfElements];

      double time = 0.0;
      for (int i = 0; i < numberOfElements; i++)
      {
         double input = Math.sin(2.0 * Math.PI * frequency * time);

         inputCurve[0][i] = time;
         inputCurve[1][i] = input;

         time = time + deltaT;
      }

      return inputCurve;
   }

   private static double[][] getFilteredCurve(double[][] input, ButterworthFilteredYoVariable butterworthFilteredYoVariable)
   {
      double[][] filteredCurve = new double[2][input[0].length];

      butterworthFilteredYoVariable.reset();

      for (int i = 0; i < input[0].length; i++)
      {
         butterworthFilteredYoVariable.update(input[1][i]);

         filteredCurve[0][i] = input[0][i];

         filteredCurve[1][i] = butterworthFilteredYoVariable.getDoubleValue();
      }

      return filteredCurve;
   }

   private static double plotBodeForAlpha(double alpha, double deltaT)
   {
      YoRegistry registry = new YoRegistry("Test");

      ButterworthFilteredYoVariable butterworthFilteredYoVariable = new ButterworthFilteredYoVariable("test", registry, alpha, ButterworthFilterType.LOW_PASS);

      double startFreq = 1e-2;
      double endFreq = 0.25 * (1.0 / deltaT);

      int numberOfTestPoints = 100;

      double deltaFreq = (endFreq - startFreq) / (numberOfTestPoints - 1);

      ArrayList<Double> testFrequencies = new ArrayList<Double>();
      ArrayList<Double> attenuations = new ArrayList<Double>();

      for (double freq = startFreq; freq <= endFreq; freq = freq + deltaFreq)
      {
         int numberOfCycles = 3;
         double endTime = (numberOfCycles) / freq;

         double[][] inputCurve = generateInputCurve(endTime, freq, deltaT);
         double[][] outputCurve = getFilteredCurve(inputCurve, butterworthFilteredYoVariable);

         double dB = getMagnitudeInDecibels(inputCurve, outputCurve);

         testFrequencies.add(freq);
         attenuations.add(dB);
      }

      @SuppressWarnings("unused")
      ArrayList<Double> testFrequenciesLog = convertToLog(testFrequencies);

      double breakFreq = getBreakFreq(testFrequencies, attenuations);

      return breakFreq;
   }

   public static double computeAlphaGivenBreakFrequency(double breakFrequencyInHetrz, double dt)
   {
      double alpha = 1.0 - 4.0 * dt * breakFrequencyInHetrz;

      alpha = MathTools.clamp(alpha, 0.0, 1.0);

      return alpha;
   }

   private static double getBreakFreq(ArrayList<Double> freq, ArrayList<Double> attenutation_dB)
   {
      for (int i = 0; i < freq.size(); i++)
      {
         double attenuation = attenutation_dB.get(i);

         if (attenuation < -3.0)
            return freq.get(i);
      }

      return Double.POSITIVE_INFINITY;
   }

   private static ArrayList<Double> convertToLog(ArrayList<Double> arrayListToConvert)
   {
      ArrayList<Double> ret = new ArrayList<Double>();

      for (Double value : arrayListToConvert)
      {
         double logValue = Math.log10(value);
         ret.add(logValue);
      }

      return ret;
   }

   private static double getMagnitudeInDecibels(double[][] input, double[][] output)
   {
      double inputAmp = getMaximumPeaktoPeakAmplitude(input);
      double outputAmp = getMaximumPeaktoPeakAmplitude(output);

      double attenuation = outputAmp / inputAmp;

      //    System.out.println("attenuation=" + attenuation);

      double dB = 20.0 * Math.log10(attenuation);

      return dB;
   }

   private static double getMaximumPeaktoPeakAmplitude(double[][] curveVersusTime)
   {
      double maximumValue = Double.NEGATIVE_INFINITY;
      double minimumValue = Double.POSITIVE_INFINITY;

      for (int i = 0; i < curveVersusTime[0].length; i++)
      {
         double value = curveVersusTime[1][i];
         if (value > maximumValue)
            maximumValue = value;

         if (value < minimumValue)
            minimumValue = value;
      }

      return maximumValue - minimumValue;
   }

   public static void main(String[] args)
   {
      ArrayList<Double> alphas = new ArrayList<Double>();
      ArrayList<Double> breakFreqs = new ArrayList<Double>();

      ArrayList<Double> alphasCalculated = new ArrayList<Double>();

      double deltaT = 0.002;

      for (double alpha = 0.1; alpha <= 0.9; alpha = alpha + 0.1)
      {
         double breakFreq = plotBodeForAlpha(alpha, deltaT);
         alphas.add(alpha);
         breakFreqs.add(breakFreq);

         double alphaCalculated = computeAlphaGivenBreakFrequency(breakFreq, deltaT);
         alphasCalculated.add(alphaCalculated);
         System.out.println("alpha= " + alpha + " , breakFreq= " + breakFreq + ", calculated alpha = " + alphaCalculated);
      }

      double[][] alphaActual = new double[2][alphas.size()];
      double[][] alphaPredicted = new double[2][alphasCalculated.size()];

      for (int i = 0; i < alphas.size(); i++)
      {
         alphaActual[0][i] = breakFreqs.get(i);
         alphaActual[1][i] = alphas.get(i);

         alphaPredicted[0][i] = breakFreqs.get(i);
         alphaPredicted[1][i] = alphasCalculated.get(i);
      }

      ArrayList<double[][]> listOfXYCurves = new ArrayList<double[][]>();

      listOfXYCurves.add(alphaActual);
      listOfXYCurves.add(alphaPredicted);
   }

}
