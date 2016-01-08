package us.ihmc.sensorProcessing.diagnostic;

import edu.emory.mathcs.jtransforms.fft.DoubleFFT_1D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredIntegerYoVariable;

public class Online1DSignalFrequencyAnalysis
{
   private final YoVariableRegistry registry;

   private final GlitchFilteredIntegerYoVariable principalOscillationIndex;
   private final GlitchFilteredIntegerYoVariable secondaryOscillationIndex;

   private final DoubleYoVariable principalFrequency;
   private final DoubleYoVariable secondaryFrequency;

   private final DoubleYoVariable principalMagnitude;
   private final DoubleYoVariable secondaryMagnitude;

   private final IntegerYoVariable minFrequencyIndex;
   private final DoubleYoVariable minimumMagnitude;

   private final DoubleFFT_1D fft;
   private final double[] signalBuffer;
   private final double[] fftOuput;
   private final double[] frequencies;
   private final double[] magnitudes;
   private final double[] filteredMagnitudes;

   private final DoubleYoVariable magnitudeAlpha;

   private boolean hasBufferBeenFilled = false;
   private int bufferPosition = 0;

   private final int numberOfObservations;
   private final double observationDuration;

   private final double dt;

   public Online1DSignalFrequencyAnalysis(String namePrefix, double estimationWindow, double dt, YoVariableRegistry parentRegistry)
   {
      this.dt = dt;

      registry = new YoVariableRegistry(namePrefix + "FrequencyAnalysis");
      parentRegistry.addChild(registry);

      int windowSize = 10;
      principalOscillationIndex = new GlitchFilteredIntegerYoVariable(namePrefix + "PrincipalOscillationIndex", windowSize, registry);
      secondaryOscillationIndex = new GlitchFilteredIntegerYoVariable(namePrefix + "SecondaryOscillationIndex", windowSize, registry);

      principalFrequency = new DoubleYoVariable(namePrefix + "PrincipalFrequency", registry);
      secondaryFrequency = new DoubleYoVariable(namePrefix + "SecondaryFrequency", registry);

      principalMagnitude = new DoubleYoVariable(namePrefix + "PrincipalMagnitude", registry);
      secondaryMagnitude = new DoubleYoVariable(namePrefix + "SecondaryMagnitude", registry);

      minFrequencyIndex = new IntegerYoVariable(namePrefix + "MinFrequencyIndex", registry);
      minFrequencyIndex.set(2);
      minimumMagnitude = new DoubleYoVariable(namePrefix + "MinimumMagnitude", registry);

      this.numberOfObservations = (int) (estimationWindow / dt);
      observationDuration = (numberOfObservations - 1) * dt;

      fft = new DoubleFFT_1D(numberOfObservations);
      signalBuffer = new double[numberOfObservations];
      fftOuput = new double[numberOfObservations];
      frequencies = new double[numberOfObservations / 2];
      magnitudes = new double[numberOfObservations / 2];
      filteredMagnitudes = new double[numberOfObservations / 2];

      magnitudeAlpha = new DoubleYoVariable(namePrefix + "MagnitudeAlpha", registry);

      for (int i = 0; i < numberOfObservations / 2; i++)
         frequencies[i] = i / observationDuration;
   }

   public void setMagnitudeFilterBreakFrequency(double breakFrequency)
   {
      magnitudeAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency, dt));
   }

   public void setFrequencyGlitchFilterWindow(double window)
   {
      principalOscillationIndex.setWindowSize((int) (window / dt));
      secondaryOscillationIndex.setWindowSize((int) (window / dt));
   }

   public void setMinimumMagnitude(double minimumMagnitude)
   {
      this.minimumMagnitude.set(minimumMagnitude);
   }

   public void update(double currentSignalValue)
   {
      signalBuffer[bufferPosition] = currentSignalValue;
      bufferPosition++;

      if (bufferPosition >= numberOfObservations)
      {
         bufferPosition = 0;
         hasBufferBeenFilled = true;
      }

      if (!hasBufferBeenFilled)
         return;

      for (int i = 0; i < numberOfObservations; i++)
      {
         fftOuput[i] = signalBuffer[(bufferPosition + i + 1) % numberOfObservations];
      }

      fft.realForward(fftOuput);

      double principalMagnitude = Double.NEGATIVE_INFINITY;
      int principalIndex = -1;
      double secondaryMagnitude = Double.NEGATIVE_INFINITY;
      int secondaryIndex = -1;

      for (int k = 1; k < numberOfObservations / 2; k++)
      {
         double magnitude;

         if (k >= minFrequencyIndex.getIntegerValue())
         {
            double real = fftOuput[2 * k];
            double imag = fftOuput[2 * k + 1];

            magnitude = Math.sqrt(real * real + imag * imag) / (0.5 * numberOfObservations);
         }
         else
         {
            magnitude = 0.0;
         }

         magnitudes[k] = magnitude;
         double previousFilteredMagnitude = filteredMagnitudes[k];
         double currentFilteredMagnitude = magnitudeAlpha.getDoubleValue() * previousFilteredMagnitude + (1.0 - magnitudeAlpha.getDoubleValue()) * magnitude;
         filteredMagnitudes[k] = currentFilteredMagnitude;
      }

      for (int k = 1; k < numberOfObservations / 2; k++)
      {
         double magnitude = filteredMagnitudes[k];

         if (magnitude > principalMagnitude)
         {
            secondaryMagnitude = principalMagnitude;
            secondaryIndex = principalIndex;

            principalMagnitude = magnitude;
            if (magnitude <= minimumMagnitude.getDoubleValue())
               principalIndex = -1;
            else
               principalIndex = k;
         }
         else if (magnitude > secondaryMagnitude)
         {
            secondaryMagnitude = magnitude;
            if (magnitude <= minimumMagnitude.getDoubleValue())
               secondaryIndex = -1;
            else
               secondaryIndex = k;
         }
      }

      this.principalOscillationIndex.update(principalIndex);
      this.secondaryOscillationIndex.update(secondaryIndex);

      if (principalOscillationIndex.getIntegerValue() == -1)
      {
         this.principalFrequency.set(0.0);
         this.principalMagnitude.set(0.0);
      }
      else
      {
         this.principalFrequency.set(frequencies[principalOscillationIndex.getIntegerValue()]);
         this.principalMagnitude.set(filteredMagnitudes[principalOscillationIndex.getIntegerValue()]);
      }

      if (secondaryOscillationIndex.getIntegerValue() == -1)
      {
         this.secondaryFrequency.set(0.0);
         this.secondaryMagnitude.set(0.0);
      }
      else
      {
         this.secondaryFrequency.set(frequencies[secondaryOscillationIndex.getIntegerValue()]);
         this.secondaryMagnitude.set(filteredMagnitudes[secondaryOscillationIndex.getIntegerValue()]);
      }
   }

   public boolean hasAnalysisStarted()
   {
      return hasBufferBeenFilled;
   }
}
