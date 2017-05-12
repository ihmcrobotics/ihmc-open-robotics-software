package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class SecondOrderFilteredYoVariableParameters
{
   private final DoubleYoVariable naturalFrequencyInHz;
   private final DoubleYoVariable dampingRatio;
   private final SecondOrderFilterType filterType;

   public SecondOrderFilteredYoVariableParameters(String name, YoVariableRegistry registry, double naturalFrequencyInHz, double dampingRatio, SecondOrderFilterType filterType)
   {
      this.naturalFrequencyInHz = new DoubleYoVariable(name + "NaturalFrequency", registry);
      this.naturalFrequencyInHz.set(naturalFrequencyInHz);
      this.dampingRatio = new DoubleYoVariable(name + "DampingRatio", registry);
      this.dampingRatio.set(dampingRatio);
      this.filterType = filterType;
   }

   public DoubleYoVariable getNaturalFrequencyInHz()
   {
      return naturalFrequencyInHz;
   }

   public DoubleYoVariable getDampingRatio()
   {
      return dampingRatio;
   }

   public SecondOrderFilterType getFilterType()
   {
      return filterType;
   }
}
