package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class QuadratureControlEffort implements AcsellSlowSensor
{
   private final DoubleYoVariable quadratureControlEffort;
   private final double conversionFactor;
   
   public QuadratureControlEffort(String name, double conversionFactor, YoVariableRegistry registry)
   {
      this.conversionFactor = conversionFactor;
      quadratureControlEffort = new DoubleYoVariable(name + "QuadratureControlEffort", registry);
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
