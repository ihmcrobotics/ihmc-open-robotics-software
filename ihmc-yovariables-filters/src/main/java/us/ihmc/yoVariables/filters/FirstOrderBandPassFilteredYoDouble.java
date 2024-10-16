package us.ihmc.yoVariables.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FirstOrderBandPassFilteredYoDouble extends YoDouble
{
   private final YoBoolean hasBeenCalled;

   private double filterUpdateTimeOld;
   private FirstOrderFilterType filterType;

   private final YoDouble firstCutoffFrequencyHz;
   private final YoDouble secondCutoffFrequencyHz;

   private final DoubleProvider yoTime;
   private double dt;


   public enum FirstOrderFilterType
   {
       NOTCH, BAND
   }

   public FirstOrderBandPassFilteredYoDouble(String name,
                                             String description,
                                             double minPassThroughFrequency_Hz,
                                             double maxPassThroughFrequency_Hz,
                                             DoubleProvider yoTime,
                                             FirstOrderFilterType filterType,
                                             YoRegistry registry)
   {
      this(name, description, minPassThroughFrequency_Hz, maxPassThroughFrequency_Hz, yoTime, 0.0, filterType, registry);
   }

   public FirstOrderBandPassFilteredYoDouble(String name,
                                             String description,
                                             double minPassThroughFrequency_Hz,
                                             double maxPassThroughFrequency_Hz,
                                             double dt,
                                             FirstOrderFilterType filterType,
                                             YoRegistry registry)
   {
      this(name, description, minPassThroughFrequency_Hz, maxPassThroughFrequency_Hz, null, dt, filterType, registry);
   }

   private FirstOrderBandPassFilteredYoDouble(String name,
                                              String description,
                                              double minPassThroughFrequency_Hz,
                                              double maxPassThroughFrequency_Hz,
                                              DoubleProvider yoTime,
                                              double dt,
                                              FirstOrderFilterType filterType,
                                              YoRegistry registry)
   {
      super(name, description, registry);

      String firstCutoffFrequencyName, secondCuttoffFrequencyName;
      switch (filterType)
      {
         case NOTCH:
            firstCutoffFrequencyName = name + "_NotchPassStart_Hz";
            secondCuttoffFrequencyName = name + "_NotchPassEnd_Hz";
            break;
         case BAND:
            firstCutoffFrequencyName = name + "_BandPassStart_Hz";
            secondCuttoffFrequencyName = name + "_BandPassEnd_Hz";
            break;
         default:
            throw new RuntimeException("Must specify filter type notch or break");
      }

      hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(name, "", registry);
      hasBeenCalled.set(false);
      this.firstCutoffFrequencyHz = new YoDouble(firstCutoffFrequencyName, registry);
      this.firstCutoffFrequencyHz.set(minPassThroughFrequency_Hz);
      this.secondCutoffFrequencyHz = new YoDouble(secondCuttoffFrequencyName, registry);
      this.secondCutoffFrequencyHz.set(maxPassThroughFrequency_Hz);

      this.yoTime = yoTime;
      this.dt = dt;

      this.filterType = filterType;
   }

   private void checkPassband(double minPassThroughFrequency_Hz, double maxPassThroughFrequency_Hz)
   {
      if (minPassThroughFrequency_Hz > maxPassThroughFrequency_Hz)
      {
         throw new RuntimeException(
               "minPassThroughFrequency [ " + minPassThroughFrequency_Hz + " ] > maxPassThroughFrequency [ " + maxPassThroughFrequency_Hz + " ]");
      }
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public void update(double filterInput)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
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
            case NOTCH:
               filterOutput = applyNotchFilter(filterInput, dt);
               break;
            case BAND:
               filterOutput = applyBandFilter(filterInput, dt);
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

   public void setPassBand(double minPassThroughFreqHz, double maxPassThroughFreqHz)
   {
      checkPassband(minPassThroughFreqHz, maxPassThroughFreqHz);

      firstCutoffFrequencyHz.set(minPassThroughFreqHz);
      secondCutoffFrequencyHz.set(maxPassThroughFreqHz);
   }

   private double applyNotchFilter(double filterInput, double dt)
   {
      double lowFiltered = applyLowPassFilter(filterInput, firstCutoffFrequencyHz.getDoubleValue(), dt);
      double highFiltered = applyHighPassFilter(filterInput, secondCutoffFrequencyHz.getDoubleValue(), dt);

      return lowFiltered + highFiltered;
   }

   private double applyBandFilter(double filterInput, double dt)
   {
      double lowFiltered = applyLowPassFilter(filterInput, firstCutoffFrequencyHz.getDoubleValue(), dt);
      double highFiltered = applyHighPassFilter(filterInput, secondCutoffFrequencyHz.getDoubleValue(), dt);

      return filterInput - lowFiltered - highFiltered;
   }

   private double applyLowPassFilter(double filterInput, double breakFrequency, double dt)
   {
      double alpha = AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(dt, breakFrequency);

      return alpha * this.getDoubleValue() + (1.0 - alpha) * filterInput;
   }

   private double applyHighPassFilter(double filterInput,  double breakFrequency, double dt)
   {
      double lowPassValue = applyLowPassFilter(filterInput, breakFrequency, dt);

      return filterInput - lowPassValue;
   }
}