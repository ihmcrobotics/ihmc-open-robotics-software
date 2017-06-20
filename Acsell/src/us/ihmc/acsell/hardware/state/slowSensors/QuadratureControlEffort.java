package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadratureControlEffort implements AcsellSlowSensor
{
   private final YoDouble quadratureControlEffort;
   private final double conversionFactor;
   
   public QuadratureControlEffort(String name, double conversionFactor, YoVariableRegistry registry)
   {
      this.conversionFactor = conversionFactor;
      quadratureControlEffort = new YoDouble(name + "QuadratureControlEffort", registry);
   }

   @Override
   public void update(int value)
   {
      quadratureControlEffort.set(((double)((short) value)) / conversionFactor); // / 100.0);
   }

   public double getValue()
   {
	   return quadratureControlEffort.getValueAsDouble();
   }
}
