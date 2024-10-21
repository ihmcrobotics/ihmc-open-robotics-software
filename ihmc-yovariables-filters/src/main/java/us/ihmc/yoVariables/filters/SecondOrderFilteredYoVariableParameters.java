package us.ihmc.yoVariables.filters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SecondOrderFilteredYoVariableParameters
{
   private final YoDouble naturalFrequencyInHz;
   private final YoDouble dampingRatio;
   private final SecondOrderFilteredYoDouble.SecondOrderFilterType filterType;

   public SecondOrderFilteredYoVariableParameters(String name, YoRegistry registry, double naturalFrequencyInHz, double dampingRatio,
         SecondOrderFilteredYoDouble.SecondOrderFilterType filterType)
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

   public SecondOrderFilteredYoDouble.SecondOrderFilterType getFilterType()
   {
      return filterType;
   }
}
