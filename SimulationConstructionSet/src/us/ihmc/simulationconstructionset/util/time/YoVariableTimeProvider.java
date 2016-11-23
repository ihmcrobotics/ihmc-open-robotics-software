package us.ihmc.simulationconstructionset.util.time;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.time.TimeProvider;

public class YoVariableTimeProvider implements TimeProvider
{
   private final DoubleYoVariable timeYoVariable;
   
   public YoVariableTimeProvider(DoubleYoVariable timeYoVariable)
   {
      this.timeYoVariable = timeYoVariable;
   }
   
   @Override
   public double now()
   {
      return timeYoVariable.getDoubleValue();
   }
}
