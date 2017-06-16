package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class InphaseControlEffort implements AcsellSlowSensor
{
   private final YoDouble inphaseControlEffort;
   private final double conversionFactor;
   
   public InphaseControlEffort(String name, double conversionFactor, YoVariableRegistry registry)
   {
      inphaseControlEffort = new YoDouble(name + "InphaseControlEffort", registry);
      this.conversionFactor = conversionFactor;
   }

   @Override
   public void update(int value)
   {
      inphaseControlEffort.set(((double)((short) value)) / conversionFactor);// / 100.0);
   }
   
   public double getValue()
   {
	   return inphaseControlEffort.getDoubleValue();
   }

}
