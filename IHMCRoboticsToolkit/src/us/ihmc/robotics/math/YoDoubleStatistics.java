package us.ihmc.robotics.math;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class YoDoubleStatistics
{
   private final DoubleYoVariable variable;
   private final YoVariableRegistry registry;
   
   private DoubleYoVariable min;
   private DoubleYoVariable max;
   private DoubleYoVariable average;
   private DoubleYoVariable delta;
   
   private long ticks = 0;
   private double total = 0;
   
   public YoDoubleStatistics(DoubleYoVariable variable, YoVariableRegistry registry)
   {
      this.variable = variable;
      this.registry = registry;
      
      variable.addVariableChangedListener(this::update);
   }
   
   private void update(YoVariable<?> v)
   {
      double value = variable.getDoubleValue();
      
      if (min != null && value < min.getDoubleValue())
      {
         min.set(value);
      }
      
      if (max != null && value > max.getDoubleValue())
      {
         max.set(value);
      }
      
      if (average != null)
      {
         ++ticks;
         total += value;
         average.set(total / ticks);
      }
      
      if (delta != null && min != null && max != null)
      {
         delta.set(max.getDoubleValue() - min.getDoubleValue());
      }
   }
   
   public void enableMin()
   {
      min = new DoubleYoVariable(variable.getName() + "Min", registry);
      min.set(Double.MAX_VALUE);
   }
   
   public void enableMax()
   {
      max = new DoubleYoVariable(variable.getName() + "Max", registry);
      max.set(Double.MIN_VALUE);
   }
   
   public void enableAverage()
   {
      average = new DoubleYoVariable(variable.getName() + "Average", registry);
      average.set(Double.NaN);
   }
   
   public void enableDelta()
   {
      delta = new DoubleYoVariable(variable.getName() + "Delta", registry);
      delta.set(Double.NaN);
   }
}
