package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.IntegerYoVariable;

public class GlitchFilteredYoBoolean extends YoBoolean
{
   private YoBoolean variableToFilter;
   private final IntegerYoVariable windowSize;
   protected final IntegerYoVariable counter;

   public GlitchFilteredYoBoolean(String name, int windowSize)
   {
      super(name, "GlitchFilteredYoBoolean", null);
      counter = new IntegerYoVariable(name + "Count", getYoVariableRegistry());
      this.windowSize = new IntegerYoVariable(name + "WindowSize", getYoVariableRegistry());

      initialize(null, windowSize);
   }

   public GlitchFilteredYoBoolean(String name, YoVariableRegistry registry, int windowSize)
   {
      super(name, registry);
      counter = new IntegerYoVariable(name + "Count", registry);
      this.windowSize = new IntegerYoVariable(name + "WindowSize", registry);

      initialize(null, windowSize);
   }

   public GlitchFilteredYoBoolean(String name, YoBoolean yoVariableToFilter, int windowSize)
   {
      super(name, "GlitchFilteredYoBoolean", null);
      counter = new IntegerYoVariable(name + "Count", getYoVariableRegistry());
      this.windowSize = new IntegerYoVariable(name + "WindowSize", getYoVariableRegistry());

      initialize(yoVariableToFilter, windowSize);
   }

   public GlitchFilteredYoBoolean(String name, YoVariableRegistry registry, YoBoolean yoVariableToFilter, int windowSize)
   {
      this(name, "", registry, yoVariableToFilter, windowSize);
   }

   public GlitchFilteredYoBoolean(String name, String description, YoVariableRegistry registry, YoBoolean yoVariableToFilter, int windowSize)
   {
      super(name, description, registry);
      counter = new IntegerYoVariable(name + "Count", description, registry);
      this.windowSize = new IntegerYoVariable(name + "WindowSize", description, registry);

      initialize(yoVariableToFilter, windowSize);
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

   public void set(boolean value)
   {
      super.set(value);
      if (counter != null)
      {
         counter.set(0);
      }
   }

   public void update(boolean value)
   {
	   
      if (value != this.getBooleanValue())
      {
         counter.set(counter.getIntegerValue() + 1);
      }
      else
         counter.set(0);

      if (counter.getIntegerValue() >= (windowSize.getIntegerValue() ))
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