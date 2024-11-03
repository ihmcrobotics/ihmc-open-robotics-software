package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public class GlitchFilteredYoBoolean extends YoBoolean
{
   private YoBoolean variableToFilter;
   private final YoInteger windowSize;
   protected final YoInteger counter;

   public GlitchFilteredYoBoolean(String name, int windowSize)
   {
      super(name, "GlitchFilteredYoBoolean", null);
      counter = new YoInteger(name + "Count", getRegistry());
      this.windowSize = new YoInteger(name + "WindowSize", getRegistry());

      initialize(null, windowSize);
   }

   public GlitchFilteredYoBoolean(String name, YoRegistry registry, int windowSize)
   {
      super(name, registry);
      counter = new YoInteger(name + "Count", registry);
      this.windowSize = new YoInteger(name + "WindowSize", registry);

      initialize(null, windowSize);
   }

   public GlitchFilteredYoBoolean(String name, YoBoolean yoVariableToFilter, int windowSize)
   {
      super(name, "GlitchFilteredYoBoolean", null);
      counter = new YoInteger(name + "Count", getRegistry());
      this.windowSize = new YoInteger(name + "WindowSize", getRegistry());

      initialize(yoVariableToFilter, windowSize);
   }

   public GlitchFilteredYoBoolean(String name, YoRegistry registry, YoBoolean yoVariableToFilter, int windowSize)
   {
      this(name, "", registry, yoVariableToFilter, windowSize);
   }

   public GlitchFilteredYoBoolean(String name, YoRegistry registry, YoBoolean yoVariableToFilter, YoInteger windowSize)
   {
      this(name, "", registry, yoVariableToFilter, windowSize);
   }

   public GlitchFilteredYoBoolean(String name, String description, YoRegistry registry, YoBoolean yoVariableToFilter, int windowSize)
   {
      this(name, description, registry, yoVariableToFilter, new YoInteger(name + "WindowSize", description, registry));

      this.windowSize.set(windowSize);
   }

   public GlitchFilteredYoBoolean(String name, String description, YoRegistry registry, YoBoolean yoVariableToFilter, YoInteger windowSize)
   {
      super(name, description, registry);
      counter = new YoInteger(name + "Count", description, registry);
      this.windowSize = windowSize;

      initialize(yoVariableToFilter, windowSize.getIntegerValue());
   }

   private void initialize(YoBoolean yoVariableToFilter, int windowSize)
   {
      if (windowSize < 0)
         throw new RuntimeException("window size must be greater than 0");

      variableToFilter = yoVariableToFilter;
      this.windowSize.set(windowSize);

      if (variableToFilter != null)
         this.set(yoVariableToFilter.getBooleanValue());

      this.set(false);
   }

   public boolean set(boolean value)
   {
      if (counter != null)
      {
         counter.set(0);
      }
      return super.set(value);
   }

   public void update(boolean value)
   {

      if (value != this.getBooleanValue())
      {
         counter.set(counter.getIntegerValue() + 1);
      }
      else
         counter.set(0);

      if (counter.getIntegerValue() >= (windowSize.getIntegerValue()))
         set(value);
   }

   public int getWindowSize()
   {
      return windowSize.getIntegerValue();
   }

   public void setWindowSize(int windowSize) //untested
   {
      this.windowSize.set(windowSize);
   }

   public void update()
   {
      if (variableToFilter == null)
      {
         throw new RuntimeException("variableToFilter was not initialized. Use the other constructor");
      }
      else
         update(variableToFilter.getBooleanValue());
   }
}