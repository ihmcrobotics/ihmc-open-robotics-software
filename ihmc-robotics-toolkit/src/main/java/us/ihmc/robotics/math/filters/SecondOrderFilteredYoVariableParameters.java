package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SecondOrderFilteredYoVariableParameters
{
   private final YoDouble naturalFrequencyInHz;
   private final YoDouble dampingRatio;
   private final SecondOrderFilterType filterType;

   public SecondOrderFilteredYoVariableParameters(String name, YoVariableRegistry registry, double naturalFrequencyInHz, double dampingRatio,
         SecondOrderFilterType filterType)
   {
      this.naturalFrequencyInHz = new YoDouble(name + "NaturalFrequency", registry);
      this.naturalFrequencyInHz.set(naturalFrequencyInHz);
      this.dampingRatio = new YoDouble(name + "DampingRatio", registry);
      this.dampingRatio.set(dampingRatio);
      this.filterType = filterType;
   }

   public YoDouble getNaturalFrequencyInHz()
   {
      return naturalFrequencyInHz;
   }

   public YoDouble getDampingRatio()
   {
      return dampingRatio;
   }

   public SecondOrderFilterType getFilterType()
   {
      return filterType;
   }
}
