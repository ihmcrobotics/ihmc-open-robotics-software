package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class FirstOrderBandPassFilteredYoVariable extends FirstOrderFilteredYoVariable
{
   private boolean hasBeenCalled = false;

   private final FirstOrderFilteredYoVariable highPassFilteredInput;

   public FirstOrderBandPassFilteredYoVariable(String name, String description, double minPassThroughFrequency_Hz, double maxPassThroughFrequency_Hz,
         DoubleYoVariable yoTime, YoVariableRegistry registry)
   {
      super(name, description, maxPassThroughFrequency_Hz, yoTime, FirstOrderFilterType.LOW_PASS, registry);

      this.highPassFilteredInput = new FirstOrderFilteredYoVariable(name + "HighPassFilteredOnly", description, minPassThroughFrequency_Hz, yoTime,
            FirstOrderFilterType.HIGH_PASS, registry);

      setPassBand(minPassThroughFrequency_Hz, maxPassThroughFrequency_Hz);
   }
   
   public FirstOrderBandPassFilteredYoVariable(String name, String description, double minPassThroughFrequency_Hz, double maxPassThroughFrequency_Hz,
         double DT, YoVariableRegistry registry)   
   {
      super(name, description, maxPassThroughFrequency_Hz, DT, FirstOrderFilterType.LOW_PASS, registry);

      this.highPassFilteredInput = new FirstOrderFilteredYoVariable(name + "HighPassFilteredOnly", description, minPassThroughFrequency_Hz, DT,
            FirstOrderFilterType.HIGH_PASS, registry);
   }

   private void checkPassband(double minPassThroughFrequency_Hz, double maxPassThroughFrequency_Hz)
   {
      if (minPassThroughFrequency_Hz > maxPassThroughFrequency_Hz)
      {
         throw new RuntimeException("minPassThroughFrequency [ " + minPassThroughFrequency_Hz + " ] > maxPassThroughFrequency [ " + maxPassThroughFrequency_Hz
               + " ]");
      }
   }

   public void update(double filterInput)
   {
      if (!hasBeenCalled)
      {
         hasBeenCalled = true;
         this.set(filterInput);
      }
      else
      {
         updateHighPassFilterAndThenLowPassFilterThat(filterInput);
      }
   }

   public void setPassBand(double minPassThroughFreqHz, double maxPassThroughFreqHz)
   {
      checkPassband(minPassThroughFreqHz, maxPassThroughFreqHz);

      highPassFilteredInput.setCutoffFrequencyHz(minPassThroughFreqHz);
      this.setCutoffFrequencyHz(maxPassThroughFreqHz);
   }

   private void updateHighPassFilterAndThenLowPassFilterThat(double filterInput)
   {
      this.highPassFilteredInput.update(filterInput);

      super.update(highPassFilteredInput.getDoubleValue());
   }
}