package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class QuadratureControlEffort implements AcsellSlowSensor
{
   private final DoubleYoVariable quadratureControlEffort;
   
   public QuadratureControlEffort(String name, YoVariableRegistry registry)
   {
      quadratureControlEffort = new DoubleYoVariable(name + "QuadratureControlEffort", registry);
   }

   @Override
   public void update(int value)
   {
      quadratureControlEffort.set(((double)((short) value)) / 100.0);
   }

   public double getValue()
   {
	   return quadratureControlEffort.getValueAsDouble();
   }
}
