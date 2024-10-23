package us.ihmc.yoVariables.filters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public class GlitchFilteredYoBoolean extends YoBoolean
{
   private final YoBoolean variableToFilter;
   private final YoInteger windowSize;
   protected final YoInteger counter;

   public GlitchFilteredYoBoolean(String name, YoRegistry registry, int windowSize)
   {
      this(name, "", registry, null, windowSize);
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
      this(name, description, registry, yoVariableToFilter, VariableTools.createWindowSizeYoInteger(name, "", windowSize, registry));
   }

   public GlitchFilteredYoBoolean(String name, String description, YoRegistry registry, YoBoolean yoVariableToFilter, YoInteger windowSize)
   {
      super(name, description, registry);
      counter = new YoInteger(name + "Count", description, registry);
      this.windowSize = windowSize;

      if (windowSize.getIntegerValue() < 0)
         throw new RuntimeException("window size must be greater than 0");

      variableToFilter = yoVariableToFilter;

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