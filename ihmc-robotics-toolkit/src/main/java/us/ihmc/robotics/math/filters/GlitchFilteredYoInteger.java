package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class GlitchFilteredYoInteger extends YoInteger
{
   private final YoInteger position;
   private final YoInteger previousPosition;
   private final IntegerProvider windowSize;
   private final YoInteger counter;

   public GlitchFilteredYoInteger(String name, int windowSize, YoRegistry registry)
   {
      this(name, windowSize, null, registry);
   }

   public GlitchFilteredYoInteger(String name, int windowSize, YoInteger position, YoRegistry registry)
   {
      super(name, GlitchFilteredYoInteger.class.getSimpleName(), registry);

      this.position = position;

      previousPosition = new YoInteger(name + "PrevValue", registry);
      counter = new YoInteger(name + "Count", registry);
      YoInteger yoWindowSize = new YoInteger(name + "WindowSize", registry);
      yoWindowSize.set(windowSize);
      this.windowSize = yoWindowSize;
   }

   public GlitchFilteredYoInteger(String name, IntegerProvider windowSize, YoRegistry registry)
   {
      this(name, windowSize, null, registry);
   }

   public GlitchFilteredYoInteger(String name, IntegerProvider windowSize, YoInteger position, YoRegistry registry)
   {
      super(name, GlitchFilteredYoInteger.class.getSimpleName(), registry);

      this.position = position;

      previousPosition = new YoInteger(name + "PrevValue", registry);
      counter = new YoInteger(name + "Count", registry);
      this.windowSize = windowSize;
   }

   @Override
   public boolean set(int value)
   {
      if (counter != null)
         counter.set(0);
      return super.set(value);
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

      if (counter.getIntegerValue() >= windowSize.getValue())
      {
         set(currentValue);
         counter.set(0);
      }

      previousPosition.set(currentValue);
   }

   public int getWindowSize()
   {
      return windowSize.getValue();
   }

   public void setWindowSize(int windowSize)
   {
      if (this.windowSize instanceof YoInteger)
      {
         ((YoInteger) this.windowSize).set(windowSize);
      }
      else
      {
         throw new RuntimeException("Setting the window size is not supported");
      }
   }
}
