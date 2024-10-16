package us.ihmc.yoVariables.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FirstOrderFilteredYoDouble extends YoDouble
{
   public enum FirstOrderFilterType
   {
      LOW_PASS, NOTCH, BAND, HIGH_PASS
   }

   private boolean hasBeenCalled = false;

   private double filterUpdateTimeOld;

   private final YoDouble cutoffFrequencyHz;

   private final DoubleProvider yoTime;
   private double dt;

   private FirstOrderFilterType filterType;

   public FirstOrderFilteredYoDouble(String name,
                                     String description,
                                     double cutoffFrequencyHz,
                                     DoubleProvider yoTime,
                                     FirstOrderFilterType highOrLowPass,
                                     YoRegistry registry)
   {
      super(name, description, registry);

      String cutoffFrequencyName;
      switch (highOrLowPass)
      {
         case LOW_PASS:
            cutoffFrequencyName = name + "_LowPassCutoff_Hz";
            break;
         case HIGH_PASS:
            cutoffFrequencyName = name + "_HighPassCutoff_Hz";
            break;
         default:
            throw new RuntimeException("Must Specify Filter Type as Low or High Pass.  Current Specification : " + highOrLowPass);
      }

      this.cutoffFrequencyHz = new YoDouble(cutoffFrequencyName, registry);
      this.cutoffFrequencyHz.set(cutoffFrequencyHz);

      this.yoTime = yoTime;

      this.filterType = highOrLowPass;
   }

   public FirstOrderFilteredYoDouble(String name,
                                     String description,
                                     double cutoffFrequency_Hz,
                                     double dt,
                                     FirstOrderFilterType highOrLowPass,
                                     YoRegistry registry)
   {
      this(name, description, cutoffFrequency_Hz, null, highOrLowPass, registry);
      this.dt = dt;
   }

   private double computeLowPassUpdate(double filterInput, double dt)
   {
      double alpha = AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(dt, cutoffFrequencyHz.getDoubleValue());

      return alpha * this.getDoubleValue() + (1.0 - alpha) * filterInput;
   }

   private double computeHighPassUpdate(double filterInput, double dt)
   {
      double lowPassValue = computeLowPassUpdate(filterInput, dt);

      return filterInput - lowPassValue;
   }

   public void reset()
   {
      hasBeenCalled = false;
   }

   public void setCutoffFrequencyHz(double cutoffHz)
   {
      this.cutoffFrequencyHz.set(cutoffHz);
   }

   public void update(double filterInput)
   {
      if (!hasBeenCalled)
      {
         hasBeenCalled = true;

         filterUpdateTimeOld = 0.0;

         this.set(filterInput);
      }
      else
      {
         if (yoTime != null)
         {
            double timeSinceLastUpdate = yoTime.getValue() - filterUpdateTimeOld;

            if (timeSinceLastUpdate > 0.0)
            {
               dt = timeSinceLastUpdate;
            }
            else
            {
               reset();
               //                  throw new RuntimeException("Computed step size, DT must be greater than zero.   DT = " + dt + ".  Current time = " + yoTime.getDoubleValue() + ", previous update time = " + filterUpdateTimeOld);
            }
         }

         double filterOutput;

         switch (filterType)
         {
            case LOW_PASS:
               filterOutput = computeLowPassUpdate(filterInput, dt);
               break;
            case HIGH_PASS:
               filterOutput = computeHighPassUpdate(filterInput, dt);
               break;
            default:
               throw new RuntimeException("The first order filter must be either a high pass or low pass filter, it cannot be " + filterType);
         }

         this.set(filterOutput);
      }


      if (yoTime != null)
      {
         filterUpdateTimeOld = yoTime.getValue();
      }
   }
}