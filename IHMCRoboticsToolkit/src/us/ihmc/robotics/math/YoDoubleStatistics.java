package us.ihmc.robotics.math;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoDoubleStatistics
{
   private final YoDouble variable;
   private final YoVariableRegistry registry;
   
   private YoDouble min;
   private YoDouble max;
   private YoDouble average;
   private YoDouble delta;
   
   private long ticks = 0;
   private double total = 0;
   
   public YoDoubleStatistics(YoDouble variable, YoVariableRegistry registry)
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
      min = new YoDouble(variable.getName() + "Min", registry);
      min.set(Double.MAX_VALUE);
   }
   
   public void enableMax()
   {
      max = new YoDouble(variable.getName() + "Max", registry);
      max.set(Double.MIN_VALUE);
   }
   
   public void enableAverage()
   {
      average = new YoDouble(variable.getName() + "Average", registry);
      average.set(Double.NaN);
   }
   
   public void enableDelta()
   {
      delta = new YoDouble(variable.getName() + "Delta", registry);
      delta.set(Double.NaN);
   }
}
