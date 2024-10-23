package us.ihmc.sensorProcessing.diagnostic;

import org.ejml.data.DMatrixRMaj;
import org.jtransforms.fft.DoubleFFT_1D;

import us.ihmc.yoVariables.filters.AlphaFilterTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.filters.GlitchFilteredYoInteger;

public class Online1DSignalFourierAnalysis
{
   private final YoRegistry registry;

   private final GlitchFilteredYoInteger principalOscillationIndex;
   private final GlitchFilteredYoInteger secondaryOscillationIndex;

   private final YoDouble principalFrequency;
   private final YoDouble secondaryFrequency;

   private final YoDouble principalMagnitude;
   private final YoDouble secondaryMagnitude;

   private final YoInteger minFrequencyIndex;
   private final YoDouble minimumMagnitude;

   private final YoBoolean enabled;

   private final DoubleFFT_1D fft;
   private final double[] signalBuffer;
   private final double[] fftOuput;
   private final double[] frequencies;
   private final double[] magnitudes;
   private final double[] filteredMagnitudes;

   private final YoDouble magnitudeAlpha;

   private boolean hasBufferBeenFilled = false;
   private int bufferPosition = 0;

   private final int numberOfObservations;
   private final double observationDuration;

   private final double dt;

   public Online1DSignalFourierAnalysis(String namePrefix, double estimationWindow, double dt, YoRegistry parentRegistry)
   {
      this.dt = dt;

      registry = new YoRegistry(namePrefix + "FrequencyAnalysis");
      parentRegistry.addChild(registry);

      enabled = new YoBoolean(registry.getName() + "_enabled", registry);

      int windowSize = 10;
      principalOscillationIndex = new GlitchFilteredYoInteger(namePrefix + "PrincipalOscillationIndex", windowSize, registry);
      secondaryOscillationIndex = new GlitchFilteredYoInteger(namePrefix + "SecondaryOscillationIndex", windowSize, registry);

      principalFrequency = new YoDouble(namePrefix + "PrincipalFrequency", registry);
      secondaryFrequency = new YoDouble(namePrefix + "SecondaryFrequency", registry);

      principalMagnitude = new YoDouble(namePrefix + "PrincipalMagnitude", registry);
      secondaryMagnitude = new YoDouble(namePrefix + "SecondaryMagnitude", registry);

      minFrequencyIndex = new YoInteger(namePrefix + "MinFrequencyIndex", registry);
      minFrequencyIndex.set(2);
      minimumMagnitude = new YoDouble(namePrefix + "MinimumMagnitude", registry);

      this.numberOfObservations = (int) (estimationWindow / dt);
      observationDuration = (numberOfObservations - 1) * dt;

      fft = new DoubleFFT_1D(numberOfObservations);
      signalBuffer = new double[numberOfObservations];
      fftOuput = new double[numberOfObservations];
      frequencies = new double[numberOfObservations / 2];
      magnitudes = new double[numberOfObservations / 2];
      filteredMagnitudes = new double[numberOfObservations / 2];

      magnitudeAlpha = new YoDouble(namePrefix + "MagnitudeAlpha", registry);

      for (int i = 0; i < numberOfObservations / 2; i++)
         frequencies[i] = i / observationDuration;
   }

   public void enable()
   {
      enabled.set(true);
   }

   public void disable()
   {
      enabled.set(false);
      reset();
   }

   public void setMagnitudeFilterBreakFrequency(double breakFrequency)
   {
      magnitudeAlpha.set(AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(breakFrequency, dt));
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

   private boolean firstEstimationTick = true;

   public void update(double currentSignalValue)
   {
      if (!enabled.getBooleanValue())
         return;

      signalBuffer[bufferPosition] = currentSignalValue;
      bufferPosition++;

      if (bufferPosition >= numberOfObservations)
      {
         bufferPosition = 0;
         hasBufferBeenFilled = true;
      }

      if (!hasBufferBeenFilled)
         return;

      computeFFT();

      findPrincipalOscillations();
   }

   private void computeFFT()
   {
      for (int i = 0; i < numberOfObservations; i++)
      {
         fftOuput[i] = signalBuffer[(bufferPosition + i + 1) % numberOfObservations];
      }

      fft.realForward(fftOuput);

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
         double currentFilteredMagnitude;
         if (firstEstimationTick)
         {
            currentFilteredMagnitude = magnitude;
         }
         else
         {
            double previousFilteredMagnitude = filteredMagnitudes[k];
            currentFilteredMagnitude = magnitudeAlpha.getDoubleValue() * previousFilteredMagnitude + (1.0 - magnitudeAlpha.getDoubleValue()) * magnitude;
         }
         filteredMagnitudes[k] = currentFilteredMagnitude;
      }

      firstEstimationTick = false;
   }

   private void findPrincipalOscillations()
   {
      double principalMagnitude = Double.NEGATIVE_INFINITY;
      int principalIndex = -1;
      for (int k = 1; k < numberOfObservations / 2; k++)
      {
         double magnitude = filteredMagnitudes[k];

         if (magnitude > principalMagnitude)
         {
            principalMagnitude = magnitude;
            if (magnitude <= minimumMagnitude.getDoubleValue())
               principalIndex = -1;
            else
               principalIndex = k;
         }
      }

      principalOscillationIndex.update(principalIndex);

      if (principalOscillationIndex.getIntegerValue() == -1)
      {
         this.principalFrequency.set(0.0);
         this.principalMagnitude.set(0.0);

         this.secondaryOscillationIndex.set(-1);
         this.secondaryFrequency.set(0.0);
         this.secondaryMagnitude.set(0.0);
         return;
      }
      else
      {
         this.principalFrequency.set(frequencies[principalOscillationIndex.getIntegerValue()]);
         this.principalMagnitude.set(filteredMagnitudes[principalOscillationIndex.getIntegerValue()]);
      }

      int ignoreBandFirstIndex = principalOscillationIndex.getIntegerValue() - 1;
      int ignoreBandLastIndex = principalOscillationIndex.getIntegerValue() + 1;
      int counter = 0;

      for (int k = principalOscillationIndex.getIntegerValue() - 1; k >= 0; k--)
      {
         if (magnitudes[k] <= magnitudes[k + 1])
         {
            counter = 0;
            ignoreBandFirstIndex = k;
         }
         else
         {
            counter++;
         }

         if (counter >= 2)
            break;
      }

      counter = 0;

      for (int k = principalOscillationIndex.getIntegerValue() + 1; k < numberOfObservations / 2; k++)
      {
         if (magnitudes[k] <= magnitudes[k - 1])
         {
            counter = 0;
            ignoreBandLastIndex = k;
         }
         else
         {
            counter++;
         }

         if (counter >= 2)
            break;
      }

      double secondaryMagnitude = Double.NEGATIVE_INFINITY;
      int secondaryIndex = -1;
      for (int k = 1; k < numberOfObservations / 2; k++)
      {
         if (k > ignoreBandFirstIndex && k < ignoreBandLastIndex)
            continue;

         double magnitude = filteredMagnitudes[k];

         if (magnitude > secondaryMagnitude)
         {
            secondaryMagnitude = magnitude;
            if (magnitude <= minimumMagnitude.getDoubleValue())
               secondaryIndex = -1;
            else
               secondaryIndex = k;
         }
      }

      secondaryOscillationIndex.update(secondaryIndex);

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

   public void reset()
   {
      bufferPosition = 0;
      hasBufferBeenFilled = false;
      firstEstimationTick = true;
   }

   public boolean hasAnalysisStarted()
   {
      return hasBufferBeenFilled;
   }

   public int getOutputSize()
   {
      return frequencies.length;
   }

   public double getFrequency(int index)
   {
      return frequencies[index];
   }

   public double getMagnitude(int index)
   {
      return magnitudes[index];
   }

   /**
    * Pack in the given matrix the result of the current Fourier analysis.
    * The first column contains all the frequencies.
    * The second column contains the corresponding magnitudes.
    * @param outputToPack matrix in which the result is packed.
    */
   public void getOutput(DMatrixRMaj outputToPack)
   {
      outputToPack.reshape(frequencies.length, 2);
      for (int i = 0; i < frequencies.length; i++)
      {
         outputToPack.set(i, 0, frequencies[i]);
         outputToPack.set(i, 1, magnitudes[i]);
      }
   }

   public void getFrequencies(double[] frequenciesToPack)
   {
      System.arraycopy(frequencies, 0, frequenciesToPack, 0, frequencies.length);
   }

   public void getMagnitudes(double[] magnitudesToPack)
   {
      System.arraycopy(magnitudes, 0, magnitudesToPack, 0, magnitudes.length);
   }
}
