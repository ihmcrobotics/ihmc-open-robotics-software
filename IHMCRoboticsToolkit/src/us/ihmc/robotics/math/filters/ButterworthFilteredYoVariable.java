package us.ihmc.robotics.math.filters;

import java.util.ArrayList;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

/**
 * @author jrebula
 *         </p>
 *         <p>
 *         LittleDogVersion06:
 *         us.ihmc.LearningLocomotion.Version06.util.YoAlphaFilteredVariable,
 *         9:34:00 AM, Aug 29, 2006
 *         </p>
 *         <p>
 *         A YoAlphaFilteredVariable is a filtered version of an input YoVar.
 *         Either a YoVariable holding the unfiltered val is passed in to the
 *         constructor and update() is called every tick, or update(double) is
 *         called every tick. The YoAlphaFilteredVariable updates it's val
 *         with the current filtered version using
 *         </p>
 *         <pre>
 *            filtered_{n} = alpha * filtered_{n-1} + 1/2 * (1 - alpha) * (raw_{n} + raw{n-1}}
 *         </pre>
 */
public class ButterworthFilteredYoVariable extends DoubleYoVariable
{
   private final double alpha;
   private final DoubleYoVariable alphaVariable;

   private final DoubleYoVariable position;
   private final DoubleYoVariable previousInput;

   private final BooleanYoVariable hasBeenCalled;

   private final ButterworthFilterType butterworthFilterType;

   public ButterworthFilteredYoVariable(String name, YoVariableRegistry registry, double alpha, ButterworthFilterType butterworthFilterType)
   {
      super(name, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      this.alpha = alpha;
      this.alphaVariable = null;

      this.position = null;
      this.previousInput = new DoubleYoVariable(name + "_prevIn", registry);

      this.butterworthFilterType = butterworthFilterType;

      reset();
   }

   public ButterworthFilteredYoVariable(String name, YoVariableRegistry registry, double alpha, DoubleYoVariable positionVariable,
         ButterworthFilterType butterworthFilterType)
   {
      super(name, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      this.alpha = alpha;
      this.alphaVariable = null;

      this.position = positionVariable;
      this.previousInput = new DoubleYoVariable(name + "_prevIn", registry);

      this.butterworthFilterType = butterworthFilterType;

      reset();
   }

   public ButterworthFilteredYoVariable(String name, YoVariableRegistry registry, DoubleYoVariable alphaVariable, ButterworthFilterType butterworthFilterType)
   {
      super(name, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      this.alpha = 0.0;
      this.alphaVariable = alphaVariable;

      this.position = null;
      this.previousInput = new DoubleYoVariable(name + "_prevIn", registry);

      this.butterworthFilterType = butterworthFilterType;

      reset();
   }

   public ButterworthFilteredYoVariable(String name, YoVariableRegistry registry, DoubleYoVariable alphaVariable, DoubleYoVariable positionVariable,
         ButterworthFilterType butterworthFilterType)
   {
      super(name, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      this.alpha = 0.0;
      this.alphaVariable = alphaVariable;

      this.position = positionVariable;
      this.previousInput = new DoubleYoVariable(name + "_prevIn", registry);

      this.butterworthFilterType = butterworthFilterType;

      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public void update()
   {
      if (position == null)
      {
         throw new NullPointerException("YoButterworthFilteredVariable must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(position.getDoubleValue());
   }

   public void update(double currentInput)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);

         if (this.butterworthFilterType == ButterworthFilterType.HIGH_PASS)
         {
            set(0.0);
         }

         else
         {
            set(currentInput);
         }

         previousInput.set(currentInput);
      }

      double alphaToUse;

      if (alphaVariable == null)
      {
         alphaToUse = alpha;
      }
      else
      {
         alphaToUse = alphaVariable.getDoubleValue();
      }

      switch (butterworthFilterType)
      {
         case LOW_PASS:
         {
            set(alphaToUse * getDoubleValue() + 0.5 * (1.0 - alphaToUse) * (currentInput + previousInput.getDoubleValue()));

            break;
         }

         case HIGH_PASS:
         {
            set(alphaToUse * getDoubleValue() + 0.5 * (1.0 + alphaToUse) * (currentInput - previousInput.getDoubleValue()));

            break;
         }

      }

      previousInput.set(currentInput);
   }

   public enum ButterworthFilterType
   {
      LOW_PASS, HIGH_PASS
   }

   ;

   public static void testButterWorth()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      double alpha = .99;

      ButterworthFilteredYoVariable butterworthFilteredYoVariable = new ButterworthFilteredYoVariable("test", registry, alpha, ButterworthFilterType.LOW_PASS);

      double freq;
      int numberOfCycles;
      double endTime;
      @SuppressWarnings("unused")
      int numberOfElements;

      freq = 1.0;
      numberOfCycles = 3;

      endTime = (numberOfCycles) / freq;

      double deltaT = 0.002;
      numberOfElements = (int) Math.ceil(endTime / deltaT) + 1;

      ArrayList<double[][]> listOfXYCurves = new ArrayList<double[][]>();

      double[][] inputCurve;
      double[][] outputCurve;

      inputCurve = generateInputCurve(endTime, freq, deltaT);
      outputCurve = getFilteredCurve(inputCurve, butterworthFilteredYoVariable);

      double dB = getMagnitudeInDecibels(inputCurve, outputCurve);
      System.out.println("dB= " + dB);

      listOfXYCurves.add(inputCurve);
      listOfXYCurves.add(outputCurve);
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
      YoVariableRegistry registry = new YoVariableRegistry("Test");

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
      //    testButterWorth();

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
