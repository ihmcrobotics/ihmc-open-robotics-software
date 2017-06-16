package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class GlitchFilteredYoInteger extends YoInteger
{
   private final YoInteger position;
   private final YoInteger previousPosition;
   private final YoInteger windowSize;
   private final YoInteger counter;

   public GlitchFilteredYoInteger(String name, int windowSize, YoVariableRegistry registry)
   {
      this(name, windowSize, null, registry);
   }

   public GlitchFilteredYoInteger(String name, int windowSize, YoInteger position, YoVariableRegistry registry)
   {
      super(name, GlitchFilteredYoInteger.class.getSimpleName(), registry);

      this.position = position;

      previousPosition = new YoInteger(name + "PrevValue", registry);
      counter = new YoInteger(name + "Count", registry);
      this.windowSize = new YoInteger(name + "WindowSize", registry);
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
               "GlitchFilteredYoInteger must be constructed with a non null position variable to call update(), otherwise use update(int)");
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
