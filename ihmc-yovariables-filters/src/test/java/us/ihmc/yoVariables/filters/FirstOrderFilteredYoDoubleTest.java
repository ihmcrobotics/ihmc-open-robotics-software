package us.ihmc.yoVariables.filters;

import org.junit.jupiter.api.Test;
import us.ihmc.yoVariables.filters.FirstOrderFilteredYoDouble.FirstOrderFilterType;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class FirstOrderFilteredYoDoubleTest
{
   private final YoRegistry registry = new YoRegistry("testRegistry");
   private final YoDouble yoTime = new YoDouble("yoTime", registry);

   private final double DT = 0.001;

   @Test
   public void testHighPassAttenuationForSinusoidalInput()
   {
      double inputFrequencyRadPerSec = 15.0;
      double cutoffFrequencyRadPerSec = inputFrequencyRadPerSec / 5.0;
      double filterAttenuation = 1.0;
      double properHighPassAttenuation;

      FirstOrderFilteredYoDouble highPassFilteredYoVariable = new FirstOrderFilteredYoDouble("highPass",
                                                                                             "",
                                                                                                 cutoffFrequencyRadPerSec / (2.0 * Math.PI),
                                                                                             yoTime,
                                                                                             FirstOrderFilterType.HIGH_PASS,
                                                                                             registry);

      while (filterAttenuation > 0.1 && cutoffFrequencyRadPerSec > 0.0)
      {
         highPassFilteredYoVariable.setCutoffFrequencyHz(cutoffFrequencyRadPerSec / (2.0 * Math.PI));
         highPassFilteredYoVariable.reset();
         filterAttenuation = computeSteadyStateFilteredOutputAmplitude(yoTime, DT, inputFrequencyRadPerSec, highPassFilteredYoVariable);

         properHighPassAttenuation = computeProperHighPassAttenuation(inputFrequencyRadPerSec, cutoffFrequencyRadPerSec);

         assertEquals(properHighPassAttenuation, filterAttenuation, 1e-2);

         cutoffFrequencyRadPerSec += 10.0;
      }
   }

   @Test
   public void testLowPassAttenuationForSinusoidalInput()
   {
      double inputFrequencyRadPerSec = 10.0;
      double cutoffFrequencyRadPerSec = inputFrequencyRadPerSec * 5.0;
      double filterAttenuation = 1.0;
      double properLowPassAttenuation;

      FirstOrderFilteredYoDouble lowPassFilteredYoVariable = new FirstOrderFilteredYoDouble("lowPass",
                                                                                            "",
                                                                                                cutoffFrequencyRadPerSec / (2.0 * Math.PI),
                                                                                            yoTime,
                                                                                            FirstOrderFilterType.LOW_PASS,
                                                                                            registry);

      while (filterAttenuation > 0.1 && cutoffFrequencyRadPerSec > 0.0)
      {
         lowPassFilteredYoVariable.setCutoffFrequencyHz(cutoffFrequencyRadPerSec / (2.0 * Math.PI));
         lowPassFilteredYoVariable.reset();
         filterAttenuation = computeSteadyStateFilteredOutputAmplitude(yoTime, DT, inputFrequencyRadPerSec, lowPassFilteredYoVariable);

         properLowPassAttenuation = computeProperLowPassAttenuation(inputFrequencyRadPerSec, cutoffFrequencyRadPerSec);

         assertEquals(properLowPassAttenuation, filterAttenuation, 1e-2);

         cutoffFrequencyRadPerSec -= 10.0;
      }
   }

   @Test
   public void testBandPassAttenuationForSinusoidalInput()
   {
      double inputFrequencyRadPerSec = 10.0;

      double a = inputFrequencyRadPerSec / 5.0;
      double b = inputFrequencyRadPerSec * 5.0;

      double filterAttenuation = 1.0;
      double properBandPassAttenuation;

      FirstOrderBandPassFilteredYoDouble bandPassFilteredYoVariable = new FirstOrderBandPassFilteredYoDouble("sineWave",
                                                                                                             "",
                                                                                                             a,
                                                                                                             b,
                                                                                                             yoTime,
                                                                                                             FirstOrderBandPassFilteredYoDouble.FirstOrderFilterType.BAND,
                                                                                                             registry);

      while (filterAttenuation > 0.1 && a > 0.0 && b > 0.0)
      {
         bandPassFilteredYoVariable.setPassBand(a / (2.0 * Math.PI), b / (2.0 * Math.PI));
         bandPassFilteredYoVariable.reset();
         filterAttenuation = computeSteadyStateFilteredOutputAmplitude(yoTime, DT, inputFrequencyRadPerSec, bandPassFilteredYoVariable);

         properBandPassAttenuation = computeProperBandPassAttenuation(inputFrequencyRadPerSec, a, b);

         assertEquals(properBandPassAttenuation, filterAttenuation, 1e-2);

         a -= 10.0;
         b -= 10.0;
      }
   }

   private double computeProperLowPassAttenuation(double inputFreq_RadPerSec, double cutoffFreq_RadPerSec)
   {
      double ret = cutoffFreq_RadPerSec / Math.sqrt(inputFreq_RadPerSec * inputFreq_RadPerSec + cutoffFreq_RadPerSec * cutoffFreq_RadPerSec);
      return ret;
   }

   private double computeProperHighPassAttenuation(double inputFreq_RadPerSec, double cutoffFreq_RadPerSec)
   {
      double ret = inputFreq_RadPerSec / Math.sqrt(inputFreq_RadPerSec * inputFreq_RadPerSec + cutoffFreq_RadPerSec * cutoffFreq_RadPerSec);
      return ret;
   }

   private double computeProperBandPassAttenuation(double inputFreq_RadPerSec, double minFreq_RadPerSec, double maxFreq_RadPerSec)
   {
      double highPass = computeProperHighPassAttenuation(inputFreq_RadPerSec, minFreq_RadPerSec);
      double lowPass = computeProperLowPassAttenuation(inputFreq_RadPerSec, maxFreq_RadPerSec);

      double ret = highPass * lowPass;
      return ret;
   }

   private double computeSteadyStateFilteredOutputAmplitude(YoDouble yoTime,
                                                            double DT,
                                                            double inputFrequencyRadPerSec,
                                                            FirstOrderFilteredYoDouble filteredYoVariable)
   {
      double t;
      double sineWaveInput;

      double filterOutput_oldest = 0.0;
      double filterOutput_old = 0.0;
      double filterOutput;

      double filterOutputPeak = 0.0;
      double filterOutputPeakOld = 0.0;
      double filterOutputPeakPercentChange = 100.0;

      boolean filterOutputHasReachedSteadyState = false;

      int i = 0;

      while (!filterOutputHasReachedSteadyState)
      {
         t = i * DT;
         yoTime.set(t);

         sineWaveInput = Math.sin(inputFrequencyRadPerSec * t);

         filteredYoVariable.update(sineWaveInput);

         filterOutput = filteredYoVariable.getDoubleValue();

         boolean filterOutputJustHitAPeak = filterOutput_old > filterOutput_oldest && filterOutput_old > filterOutput;

         if (filterOutputJustHitAPeak)
         {
            filterOutputPeak = filterOutput_old;
            filterOutputPeakPercentChange = 100.0 * Math.abs((filterOutputPeak - filterOutputPeakOld) / filterOutputPeak);

            filterOutputPeakOld = filterOutputPeak;

            //            System.out.println("Filter output peak :" + filterOutputPeak);
         }

         filterOutputHasReachedSteadyState = filterOutputPeakPercentChange < 1e-6;

         filterOutput_oldest = filterOutput_old;
         filterOutput_old = filterOutput;
         i++;
      }

      //    System.out.println("Max Filter Output Percent Change: " + filterOutputPeakPercentChange);

      return filterOutputPeak;
   }

   private double computeSteadyStateFilteredOutputAmplitude(YoDouble yoTime,
                                                            double DT,
                                                            double inputFrequencyRadPerSec,
                                                            FirstOrderBandPassFilteredYoDouble filteredYoVariable)
   {
      double t;
      double sineWaveInput;

      double filterOutput_oldest = 0.0;
      double filterOutput_old = 0.0;
      double filterOutput;

      double filterOutputPeak = 0.0;
      double filterOutputPeakOld = 0.0;
      double filterOutputPeakPercentChange = 100.0;

      boolean filterOutputHasReachedSteadyState = false;

      int i = 0;

      while (!filterOutputHasReachedSteadyState)
      {
         t = i * DT;
         yoTime.set(t);

         sineWaveInput = Math.sin(inputFrequencyRadPerSec * t);

         filteredYoVariable.update(sineWaveInput);

         filterOutput = filteredYoVariable.getDoubleValue();

         boolean filterOutputJustHitAPeak = filterOutput_old > filterOutput_oldest && filterOutput_old > filterOutput;

         if (filterOutputJustHitAPeak)
         {
            filterOutputPeak = filterOutput_old;
            filterOutputPeakPercentChange = 100.0 * Math.abs((filterOutputPeak - filterOutputPeakOld) / filterOutputPeak);

            filterOutputPeakOld = filterOutputPeak;

            //            System.out.println("Filter output peak :" + filterOutputPeak);
         }

         filterOutputHasReachedSteadyState = filterOutputPeakPercentChange < 1e-6;

         filterOutput_oldest = filterOutput_old;
         filterOutput_old = filterOutput;
         i++;
      }

      //    System.out.println("Max Filter Output Percent Change: " + filterOutputPeakPercentChange);

      return filterOutputPeak;
   }
}
