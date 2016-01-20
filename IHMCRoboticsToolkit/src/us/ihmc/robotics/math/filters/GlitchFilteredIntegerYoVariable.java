package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

public class GlitchFilteredIntegerYoVariable extends IntegerYoVariable
{
   private final IntegerYoVariable position;
   private final IntegerYoVariable previousPosition;
   private final IntegerYoVariable windowSize;
   private final IntegerYoVariable counter;

   public GlitchFilteredIntegerYoVariable(String name, int windowSize, YoVariableRegistry registry)
   {
      this(name, windowSize, null, registry);
   }

   public GlitchFilteredIntegerYoVariable(String name, int windowSize, IntegerYoVariable position, YoVariableRegistry registry)
   {
      super(name, GlitchFilteredIntegerYoVariable.class.getSimpleName(), registry);

      this.position = position;

      previousPosition = new IntegerYoVariable(name + "PrevValue", registry);
      counter = new IntegerYoVariable(name + "Count", registry);
      this.windowSize = new IntegerYoVariable(name + "WindowSize", registry);
      this.windowSize.set(windowSize);
   }

   @Override
   public void set(int value)
   {
      super.set(value);
      if (counter != null)
         counter.set(0);
   }

   @Override
   public boolean set(int value, boolean notifyListeners)
   {
      if (counter != null)
         counter.set(0);
      return super.set(value, notifyListeners);
   }

   public void update()
   {
      if (position == null)
      {
         throw new NullPointerException(
               "GlitchFilteredIntegerYoVariable must be constructed with a non null position variable to call update(), otherwise use update(int)");
      }

      update(position.getIntegerValue());
   }

   public void update(int currentValue)
   {
      if (currentValue == previousPosition.getIntegerValue())
         counter.increment();
      else
         counter.set(0);

      if (counter.getIntegerValue() >= windowSize.getIntegerValue())
      {
         set(currentValue);
         counter.set(0);
      }

      previousPosition.set(currentValue);
   }

   public int getWindowSize()
   {
      return windowSize.getIntegerValue();
   }

   public void setWindowSize(int windowSize)
   {
      this.windowSize.set(windowSize);
   }
}
